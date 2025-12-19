#ifndef MULTIASTAR_ONE2ONE_H
#define MULTIASTAR_ONE2ONE_H
#include "Astar.h"

namespace RendezvousAstar {
    using AgentPtr = std::shared_ptr<Agent>;

    class One2One {
    public:
        One2One(const Eigen::Vector3d& uav_pos, const Eigen::Vector3d& ugv_pos, const ros::NodeHandle& nh)
            : nh_(nh), path_id_set_({-1, 1}), astar_(std::make_shared<Astar>()) {
            ros::NodeHandle("~").getParam("use_more_directs", use_more_directs_);
            uav_ = std::make_shared<UAV>(1, uav_pos);
            ugv_ = std::make_shared<UGV>(-1, ugv_pos);
        }
        One2One(AgentPtr& uav, AgentPtr& ugv, const ros::NodeHandle& nh, const bool use_more_directs)
            : nh_(nh), uav_(uav), ugv_(ugv), path_id_set_({uav->getID(), ugv->getID()}),
              astar_(std::make_shared<Astar>()), use_more_directs_(use_more_directs) {}
        ~One2One() = default;

        bool RendezvousCheck(const Eigen::Vector3i& point) const {
            auto node_map = NodeMap::getInstance();
            for (const auto& d : Astar::direct2d8_) {
                Eigen::Vector3i next_point = {
                    point[0] + static_cast<int32_t>(d[0]), point[1] + static_cast<int32_t>(d[1]), point[2]};
                const auto node = node_map->getNode(next_point);
                if (!NodeMap::query(next_point)) {
                    if (!node || node->getState(path_id_set_[0]) != Node::STATE::INCOMMONSET) {
                        return false;
                    }
                }
            }
            return true;
        }
        static bool Compare(
            const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2, const std::vector<int32_t>& path_id_set) {
            double power        = 0.7;
            double vf           = 0.5 * power * power + 0.001;
            double vc           = 0.3;
            double hover_weight = 1.0, move_weight = 1.5;
            double gf1 = 0.0, gc1 = 0.0;
            double gf2 = 0.0, gc2 = 0.0;
            for (const auto& id : path_id_set) {
                if (id < 0) {
                    gc1 += n1->getG(id);
                    gc2 += n2->getG(id);
                } else {
                    gf1 += n1->getG(id);
                    gf2 += n2->getG(id);
                }
            }

            const double hover_time1 = abs(1.0 * gf1 / vf - 1.0 * gc1 / vc);
            const double hover_time2 = abs(1.0 * gf2 / vf - 1.0 * gc2 / vc);

            const double moving_time1 = gf1 / vf;
            const double moving_time2 = gf2 / vf;

            const double p1 = hover_weight * hover_time1 + move_weight * moving_time1 + gc1 + gf1;
            const double p2 = hover_weight * hover_time2 + move_weight * moving_time2 + gc2 + gf2;

            return p1 < p2;
        }

        void plan(Eigen::Vector3i& rendezvous_pos) {
            std::shared_ptr<Node> rendezvous_node = nullptr;
            int cnt                               = 0;

            //--------------init----------------------------------------------------------
            astar_->setThreshold(100);


            //----------------搜索--------------------------------------------------------------------------
            const auto time_begin = std::chrono::high_resolution_clock::now();

            Astar::STATE state_uav = Astar::STATE::searching, state_ugv = Astar::STATE::searching;

            while (true) {
                if (state_ugv != Astar::STATE::reached && state_ugv != Astar::STATE::map_search_done) {
                    state_ugv =
                        astar_->runOnce(ugv_, uav_, uav_->getPos(), ugv_->getID(), path_id_set_, use_more_directs_);
                }
                if (state_uav == Astar::STATE::map_search_done) {
                    break;
                }
                state_uav = astar_->runOnce(uav_, ugv_, ugv_->getPos(), uav_->getID(), path_id_set_, use_more_directs_);
                if (endCondition(state_ugv) || endCondition(state_uav)) {
                    rendezvous_node =
                        astar_->sortCommonSet([this](const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2) {
                            return Compare(n1, n2, path_id_set_);
                        });
                    if (state_uav == Astar::STATE::reached || RendezvousCheck(rendezvous_node->getPos())) {
                        break;
                    }
                    astar_->addThresholdOneStep();
                    ++cnt;
                }
            }
            ROS_INFO("UAV state: %d,UGV state: %d", state_uav, state_ugv);
            ROS_INFO("commonSet size: %lu, cnt: %d", astar_->getCommonNum(), cnt);

            // test
            if (astar_->getCommonNum() == 0) {
                ROS_WARN("路径生成失败");
                return;
            }
            const auto time_end = std::chrono::high_resolution_clock::now();

            rendezvous_pos = rendezvous_node->getPos();
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);
        }


        // 多线程版本
        void planMultiThread(Eigen::Vector3i& rendezvous_node) {
            std::promise<std::shared_ptr<Node>> rendezvous_promise;
            std::future<std::shared_ptr<Node>> rendezvous_future;
            rendezvous_future = rendezvous_promise.get_future();

            //--------------init----------------------------------------------------------
            stop_.store(false);
            bool getRendezvous{false};


            //----------------搜索--------------------------------------------------------------------------
            const auto time_begin = std::chrono::high_resolution_clock::now();

            std::thread ugv_plan([this]() { // 改为值捕获而不是引用捕获
                ROS_INFO("ugv thread");
                const auto state_ugv = astar_->run(
                    ugv_, uav_, uav_->getPos(), ugv_->getID(), path_id_set_,
                    [this](const Astar::STATE& ugv) {
                        return stop_.load() || ugv == Astar::STATE::reached || ugv == Astar::STATE::map_search_done
                            || ugv == Astar::STATE::over_time;
                    },
                    use_more_directs_);
                ROS_INFO("UGV state: %d", state_ugv);
            });

            std::thread uav_plan([this]() { // 改为值捕获而不是引用捕获
                ROS_INFO("uav thread");
                const auto state_uav = astar_->run(
                    uav_, ugv_, ugv_->getPos(), uav_->getID(), path_id_set_,
                    [this](const Astar::STATE& uav) {
                        return stop_.load() || uav == Astar::STATE::map_search_done || uav == Astar::STATE::over_time;
                    },
                    use_more_directs_);
                if (state_uav == Astar::STATE::map_search_done) {
                    stop_.store(true);
                }
                ROS_INFO("UAV state: %d", state_uav);
            });


            std::thread monitor([this, &rendezvous_promise, &getRendezvous]() {
                while (!stop_.load()) {
                    std::vector<std::shared_ptr<Node>> snapshot;
                    {
                        std::shared_lock lock(mutex_);
                        snapshot = astar_->getCommonSet();
                    }

                    if (!snapshot.empty()) {
                        sort(snapshot.begin(), snapshot.end(),
                            [this](const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2) {
                                return Compare(n1, n2, path_id_set_);
                            });
                        const auto node = snapshot.front();
                        if (RendezvousCheck(node->getPos())) {
                            rendezvous_promise.set_value(node);
                            getRendezvous = true;
                            stop_.store(true);
                        }
                        // else {
                        //     astar_->addThresholdOneStep();
                        // }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            });

            uav_plan.join();
            ugv_plan.join();
            monitor.join();

            const auto time_end = std::chrono::high_resolution_clock::now();
            ROS_INFO("commonSet size: %lu", astar_->getCommonNum());
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);

            if (getRendezvous) {
                try {
                    ROS_INFO("获取汇合点");
                    rendezvous_node = rendezvous_future.get()->getPos();
                } catch (const std::future_error& e) {
                    ROS_WARN("路径生成失败，未得到汇合点 %s", e.what());
                    return;
                }
            } else {
                ROS_WARN("路径生成失败，未得到汇合点");
                return;
            }
        }

        static bool endCondition(const Astar::STATE& state) {
            return state == Astar::STATE::common_over_threshold || state == Astar::STATE::reached_and_common;
        }

        void visualizePath(const Eigen::Vector3i& rendezvous_node) {
            const auto path1 =
                Astar::getRealPath(uav_->getID(), uav_->getPos(), rendezvous_node, NodeMap::getInstance());
            const auto path2 =
                Astar::getRealPath(ugv_->getID(), ugv_->getPos(), rendezvous_node, NodeMap::getInstance());

            // 合并路径
            auto merged_path = path1;
            std::reverse(merged_path.begin(), merged_path.end());
            merged_path.insert(merged_path.end(), path2.begin(), path2.end());

            ROS_INFO("nodeset num: %lu", NodeMap::getInstance()->getNodeNum());

            Visualizer& visualizer = Visualizer::getInstance(nh_);
            visualizer.visualizeCommonSet(astar_->getCommonSet());
            visualizer.visualizeCommon(NodeMap::posI2D(rendezvous_node));
            visualizer.visualizePath(merged_path, 0);
        }

    private:
        ros::NodeHandle nh_;
        AgentPtr uav_;
        AgentPtr ugv_;
        const std::vector<int32_t> path_id_set_;
        std::atomic<bool> stop_{false};
        std::shared_mutex mutex_;
        std::shared_ptr<Astar> astar_;
        bool use_more_directs_ = true;
    };
} // namespace RendezvousAstar

#endif // MULTIASTAR_ONE2ONE_H
