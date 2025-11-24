#include "Astar.h"
#include <queue>
#include <ros/ros.h>
#include <thread>

#include "Ros.hpp"

namespace RendezvousAstar {
    using AgentPtr = std::shared_ptr<Agent>;

    class One2One {
    public:
        One2One(AgentPtr uav, AgentPtr ugv)
            : uav_(std::move(uav)), ugv_(std::move(ugv)), path_id_set_({uav_->getID(), ugv_->getID()}) {}
        ~One2One() = default;

        bool RendezvousCheck(const Eigen::Vector3i& point) const {
            auto node_map = NodeMap::getInstance();
            for (const auto& d : Astar::direct2d_) {
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

        std::shared_ptr<Node> getPriorityNode(std::vector<std::shared_ptr<Node>>& node_set) {

            auto compare = [this](const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2) {
                double power        = 0.7;
                double vf           = 0.5 * power * power + 0.001;
                double vc           = 0.3;
                double hover_weight = 1.0, move_weight = 1.5;
                double gf1 = 0.0, gc1 = 0.0;
                double gf2 = 0.0, gc2 = 0.0;
                for (const auto& id : path_id_set_) {
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
            };


            sort(node_set.begin(), node_set.end(), compare);
            return *node_set.begin();
        }

        void plan(const Eigen::Vector3d& uav_pos, const Eigen::Vector3d& ugv_pos, ros::NodeHandle& nh) {
            auto uav_pos_i                        = NodeMap::posD2I(uav_pos);
            auto ugv_pos_i                        = NodeMap::posD2I(ugv_pos);
            std::shared_ptr<Node> rendezvous_node = nullptr;
            int cnt                               = 0;

            //--------------init----------------------------------------------------------
            Node::version_.fetch_add(1);
            astar_.resetCommonSet();
            astar_.setThreshold(100);
            uav_->setPos(uav_pos_i);
            uav_->setInitialPos(uav_pos);
            uav_->reset(uav_->getID());

            ugv_->setPos(ugv_pos_i);
            ugv_->setInitialPos(ugv_pos);
            ugv_->reset(ugv_->getID());

            //----------------搜索--------------------------------------------------------------------------
            const auto time_begin = std::chrono::high_resolution_clock::now();

            Astar::STATE state_uav = Astar::STATE::searching, state_ugv = Astar::STATE::searching;

            while (true) {
                if (state_ugv != Astar::STATE::reached && state_ugv != Astar::STATE::map_search_done) {
                    state_ugv = astar_.runOnce(ugv_, uav_, uav_pos_i, ugv_->getID(), path_id_set_);
                }
                if (state_uav == Astar::STATE::map_search_done) {
                    break;
                }
                state_uav = astar_.runOnce(uav_, ugv_, ugv_->getPos(), uav_->getID(), path_id_set_);
                if (endCondition(state_ugv) || endCondition(state_uav)) {
                    rendezvous_node = getPriorityNode(astar_.getCommonSet());
                    if (state_uav == Astar::STATE::reached || RendezvousCheck(rendezvous_node->getPos())) {
                        break;
                    }
                    astar_.addThresholdOneStep();
                    ++cnt;
                }
            }
            ROS_INFO("UAV state: %d,UGV state: %d", state_uav, state_ugv);
            ROS_INFO("commonSet size: %lu, cnt: %d", astar_.getCommonNum(), cnt);

            // test
            if (astar_.getCommonNum() == 0) {
                ROS_WARN("路径生成失败");
                return;
            }
            const auto time_end = std::chrono::high_resolution_clock::now();
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);
            const auto path1 =
                Astar::getRealPath(uav_->getID(), uav_->getPos(), rendezvous_node->getPos(), NodeMap::getInstance());
            const auto path2 =
                Astar::getRealPath(ugv_->getID(), ugv_->getPos(), rendezvous_node->getPos(), NodeMap::getInstance());

            // 合并路径
            std::vector<Eigen::Vector3d> merged_path;
            merged_path.reserve(path1.size() + path2.size());
            merged_path.insert(merged_path.end(), path1.rbegin(), path1.rend());
            merged_path.insert(merged_path.end(), path2.begin(), path2.end());

            ROS_INFO("nodeset num: %lu", NodeMap::getInstance()->getNodeNum());

            // Config config(ros::NodeHandle("~"));
            Visualizer& visualizer = Visualizer::getInstance(nh);
            visualizer.visualizeCommonset(astar_.getCommonSet());
            visualizer.visualizeCommon(NodeMap::posI2D(rendezvous_node->getPos()));
            visualizer.visualizePath(merged_path);
        }


        // 多线程验证，说明功能库线程安全（agent不安全，但每个agent只在一个线程中）
        void planMultiThread(const Eigen::Vector3d& uav_pos, const Eigen::Vector3d& ugv_pos, ros::NodeHandle& nh) {
            auto uav_pos_i = NodeMap::posD2I(uav_pos);
            auto ugv_pos_i = NodeMap::posD2I(ugv_pos);

            //--------------init----------------------------------------------------------
            Node::version_.fetch_add(1);
            astar_.resetCommonSet();
            astar_.setThreshold(60);
            uav_->setPos(uav_pos_i);
            uav_->setInitialPos(uav_pos);
            uav_->reset(uav_->getID());

            ugv_->setPos(ugv_pos_i);
            ugv_->setInitialPos(ugv_pos);
            ugv_->reset(ugv_->getID());

            //----------------搜索--------------------------------------------------------------------------
            const auto time_begin = std::chrono::high_resolution_clock::now();

            std::thread ugv_plan([&]() {
                ROS_INFO("ugv thread");
                const auto state_ugv =
                    astar_.run(ugv_, uav_, uav_pos_i, ugv_->getID(), path_id_set_, [](const Astar::STATE& ugv) {
                        return endCondition(ugv) || ugv == Astar::STATE::reached
                            || ugv == Astar::STATE::map_search_done;
                    });
                ROS_INFO("UGV state: %d", state_ugv);
            });

            std::thread uav_plan([&]() {
                ROS_INFO("uav thread");
                const auto state_uav = astar_.run(uav_, ugv_, ugv_pos_i, uav_->getID(), path_id_set_,
                    [](const Astar::STATE& uav) { return endCondition(uav) || uav == Astar::STATE::map_search_done; });
                ROS_INFO("UAV state: %d", state_uav);
            });

            uav_plan.join();
            ugv_plan.join();
            // ROS_INFO("UAV state: %d,UGV state: %d", state_uav, state_ugv);
            ROS_INFO("commonSet size: %lu", astar_.getCommonNum());
            const auto time_end = std::chrono::high_resolution_clock::now();
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);

            auto common_point = getPriorityNode(astar_.getCommonSet())->getPos();
            const auto path1  = Astar::getRealPath(uav_->getID(), uav_->getPos(), common_point, NodeMap::getInstance());
            const auto path2  = Astar::getRealPath(ugv_->getID(), ugv_->getPos(), common_point, NodeMap::getInstance());

            // 合并路径
            auto merged_path = path1;
            std::reverse(merged_path.begin(), merged_path.end());
            merged_path.insert(merged_path.end(), path2.begin(), path2.end());

            ROS_INFO("nodeset num: %lu", NodeMap::getInstance()->getNodeNum());

            Visualizer& visualizer = Visualizer::getInstance(nh);
            visualizer.visualizeCommonset(astar_.getCommonSet());
            visualizer.visualizeCommon(NodeMap::posI2D(common_point));
            visualizer.visualizePath(merged_path);
        }

        static bool endCondition(const Astar::STATE& state) {
            return state == Astar::STATE::common_over_threshold || state == Astar::STATE::reached_and_common;
        }

    private:
        AgentPtr uav_;
        AgentPtr ugv_;
        const std::vector<int32_t> path_id_set_;
        Astar astar_;
    };
} // namespace RendezvousAstar

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "one2one_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    Visualizer::getInstance(nh);
    RendezvousAstar::PathSearch pathSearch(config, nh);


    RendezvousAstar::AgentPtr uav = std::make_shared<RendezvousAstar::UAV>();
    RendezvousAstar::AgentPtr ugv = std::make_shared<RendezvousAstar::UGV>();

    RendezvousAstar::One2One o2o(uav, ugv);

    pathSearch.setPlan([&](const Eigen::Vector3d& spos, const Eigen::Vector3d& epos) { o2o.plan(spos, epos, nh); });

    // ros::Rate lr(1000);
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     lr.sleep();
    // }

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
