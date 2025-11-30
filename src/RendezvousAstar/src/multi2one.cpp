#include "Astar.h"
#include <atomic>
#include <future>
#include <ros/ros.h>
#include <thread>
#include <unordered_map>

#include "Ros.hpp"

namespace RendezvousAstar {

    using AgentPtr = std::shared_ptr<Agent>;
    using NodePtr  = std::shared_ptr<Node>;
    class Multi2One {
    public:
        Multi2One(const std::vector<Eigen::Vector3d>& uav_pos, const std::vector<Eigen::Vector3d>& ugv_pos,
            const ros::NodeHandle& nh)
            : nh_(nh), astar_(std::make_shared<Astar>()) {
            for (int32_t i = 0; i < uav_pos.size(); ++i) {
                uavs_.emplace_back(std::make_shared<UAV>(i + 1, uav_pos[i]));
                path_id_set_.push_back(uavs_[i]->getID());
                uav_states_[uavs_[i]->getID()] = Astar::STATE::searching;
            }
            for (int32_t i = 0; i < ugv_pos.size(); ++i) {
                ugvs_.emplace_back(std::make_shared<UGV>(-(i + 1), ugv_pos[i]));
                ugv_states_[ugvs_[i]->getID()] = Astar::STATE::searching;
            }
        }
        ~Multi2One() = default;

        bool RendezvousCheck(const Eigen::Vector3i& point) const {
            auto node_map = NodeMap::getInstance();
            for (const auto& d : Astar::direct2d8_) {
                Eigen::Vector3i next_point = {
                    point[0] + static_cast<int32_t>(d[0]), point[1] + static_cast<int32_t>(d[1]), 0};
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

        // TODO: monitor应该改为两个两个检查
        void plan() {
            std::promise<std::shared_ptr<Node>> rendezvous_promise;
            std::future<std::shared_ptr<Node>> rendezvous_future = rendezvous_promise.get_future();
            bool getRendezvous{false};

            Node::version_.fetch_add(1);
            stop_.store(false);
            std::vector<std::thread> threads;

            const auto time_begin = std::chrono::high_resolution_clock::now();

            for (auto& ugv : ugvs_) {
                auto path_id_set = path_id_set_;
                path_id_set.emplace_back(ugv->getID());
                threads.emplace_back([this, &ugv, path_id_set]() {
                    const auto state = astar_->run(
                        ugv, ugv, {0, 0, 0}, ugv->getID(), path_id_set,
                        [this](const Astar::STATE& sat) {
                            return stop_.load() || sat == Astar::STATE::map_search_done
                                || sat == Astar::STATE::over_time || sat == Astar::STATE::beyond_scope;
                        },
                        false);
                    ROS_INFO("UGV id: %d  State: %d", ugv->getID(), state);
                });
            }

            for (auto& uav : uavs_) {
                auto path_id_set = path_id_set_;
                path_id_set.emplace_back(ugvs_[0]->getID());
                threads.emplace_back([this, &uav, path_id_set]() {
                    const auto state = astar_->run(
                        uav, ugvs_[0], {0, 0, 0}, uav->getID(), path_id_set,
                        [this](const Astar::STATE& sat) {
                            return stop_.load() || sat == Astar::STATE::map_search_done
                                || sat == Astar::STATE::over_time;
                        },
                        false);
                    if (state == Astar::STATE::map_search_done) {
                        stop_.store(true);
                    }
                    ROS_INFO("UAV id: %d  State: %d", uav->getID(), state);
                });
            }


            threads.emplace_back([this, &rendezvous_promise, &getRendezvous]() {
                while (!stop_.load()) {
                    std::vector<NodePtr> snapshot;
                    {
                        std::shared_lock lock(mutex_);
                        snapshot = astar_->getCommonSet();
                    }
                    if (!snapshot.empty()) {
                        auto id_set = path_id_set_;
                        id_set.emplace_back(ugvs_[0]->getID());
                        sort(snapshot.begin(), snapshot.end(),
                            [&id_set](const NodePtr& n1, const NodePtr& n2) { return Compare(n1, n2, id_set); });

                        const auto node = snapshot.front();
                        if (RendezvousCheck(node->getPos())) {
                            rendezvous_promise.set_value(node);
                            getRendezvous = true;
                            stop_.store(true);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            });

            for (auto& thread : threads) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
            Visualizer& visualizer = Visualizer::getInstance(nh_);
            visualizer.visualizeCommonSet(astar_->getCommonSet());

            // auto id_set = path_id_set_;
            // id_set.emplace_back(ugvs_[0]->getID());
            // std::sort(astar_->getCommonSet().begin(), astar_->getCommonSet().end(),
            //     [&id_set](const NodePtr& n1, const NodePtr& n2) { return Compare(n1, n2, id_set); });
            //
            // const auto rendezvous_node = rendezvous_future.get()->getPos();
            const auto time_end        = std::chrono::high_resolution_clock::now();
            ROS_INFO("commonSet size: %lu", astar_->getCommonNum());
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);
            //TODO: 不加这段就会报错，commonset内容挺正常的，但是前面调用sort后就报错了，目前不清楚什么原因。
            if (getRendezvous) {
                try {
                    ROS_INFO("获取汇合点");
                    // rendezvous_node = rendezvous_future.get()->getPos();
                } catch (const std::future_error& e) {
                    ROS_WARN("路径生成失败，未得到汇合点 %s", e.what());
                    return;
                }
            } else {
                ROS_WARN("路径生成失败，未得到汇合点");
                return;
            }


            // visualizer.visualizeCommon(NodeMap::posI2D(rendezvous_node));
            // for (const auto& uav : uavs_) {
            //     const auto path =
            //         Astar::getRealPath(uav->getID(), uav->getPos(), rendezvous_node, NodeMap::getInstance());
            //     visualizer.visualizePath(path, uav->getID());
            // }
            // visualizer.visualizePath(
            //     Astar::getRealPath(ugvs_[0]->getID(), ugvs_[0]->getPos(), rendezvous_node, NodeMap::getInstance()),
            //     ugvs_[0]->getID());
        }
        static bool endCondition(const Astar::STATE& state) {
            return state == Astar::STATE::common_over_threshold || state == Astar::STATE::reached_and_common;
        }

    private:
        ros::NodeHandle nh_;
        std::vector<AgentPtr> uavs_, ugvs_;
        std::unordered_map<int32_t, Astar::STATE> uav_states_, ugv_states_;
        std::vector<int32_t> path_id_set_;
        std::atomic<bool> stop_{false};
        std::shared_mutex mutex_;
        std::shared_ptr<Astar> astar_;
    };

} // namespace RendezvousAstar


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi2one_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    Visualizer::getInstance(nh);
    RendezvousAstar::PathSearch pathSearch(config, nh, 3, 1);

    pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
        RendezvousAstar::Multi2One multi2one(s_pos, e_pos, nh);
        multi2one.plan();
    });

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
