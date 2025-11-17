#include "Astar.h"
#include <ros/ros.h>

#include "Ros.hpp"

namespace RendezvousAstar {
    using AgentPtr = std::shared_ptr<Agent>;

    class One2One {
    public:
        One2One(AgentPtr uav, AgentPtr ugv)
            : uav_(std::move(uav)), ugv_(std::move(ugv)), path_id_set_({uav_->getID(), ugv_->getID()}) {}
        ~One2One() = default;


        void plan(const Eigen::Vector3d& uav_pos, const Eigen::Vector3d& ugv_pos, ros::NodeHandle& nh) {
            auto uav_pos_i = NodeMap::posD2I(uav_pos);
            auto ugv_pos_i = NodeMap::posD2I(ugv_pos);

            //--------------init----------------------------------------------------------
            astar_.resetCommonSet();
            for (const auto& pos : uav_->getInOpenList(uav_->getID())) {
                auto node = NodeMap::getInstance()->getNode(pos);
                if (node) {
                    node->removePath(uav_->getID());
                }
            }
            for (const auto& pos : uav_->getClosedList(uav_->getID())) {
                auto node = NodeMap::getInstance()->getNode(pos);
                if (node) {
                    node->removePath(uav_->getID());
                }
            }
            uav_->setPos(uav_pos_i);
            uav_->setInitialPos(uav_pos);
            uav_->reset(uav_->getID());
            for (const auto& pos : ugv_->getInOpenList(ugv_->getID())) {
                auto node = NodeMap::getInstance()->getNode(pos);
                if (node) {
                    node->removePath(ugv_->getID());
                }
            }
            for (const auto& pos : ugv_->getClosedList(ugv_->getID())) {
                auto node = NodeMap::getInstance()->getNode(pos);
                if (node) {
                    node->removePath(ugv_->getID());
                }
            }
            ugv_->setPos(ugv_pos_i);
            ugv_->setInitialPos(ugv_pos);
            ugv_->reset(ugv_->getID());

            //----------------搜索--------------------------------------------------------------------------
            const auto time_begin = std::chrono::high_resolution_clock::now();

            Astar::STATE state_uav = Astar::STATE::searching, state_ugv = Astar::STATE::searching;

            while (!(endCondition(state_ugv) || endCondition(state_uav))) {
                if (state_ugv != Astar::STATE::reached && state_ugv != Astar::STATE::map_search_done) {
                    state_ugv = astar_.runOnce(ugv_, uav_pos_i, NodeMap::getInstance(), ugv_->getID(), path_id_set_);
                }
                if (state_uav == Astar::STATE::reached || state_uav == Astar::STATE::map_search_done) {
                    break;
                }
                state_uav = astar_.runOnce(uav_, ugv_pos_i, NodeMap::getInstance(), uav_->getID(), path_id_set_);
            }
            ROS_INFO("UAV state: %d,UGV state: %d", state_uav, state_ugv);
            ROS_INFO("commonSet size: %lu", astar_.getCommonNum());
            const auto time_end = std::chrono::high_resolution_clock::now();
            ROS_INFO("one2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);

            // test
            // TODO:双向搜索验证完毕，可进行后续最优点选取部分编写。
            auto common_point = *astar_.getCommonSet().begin();
            auto common_node  = NodeMap::getInstance()->getNode(common_point);
            const auto path1  = Astar::getRealPath(uav_->getID(), uav_->getPos(), common_point, NodeMap::getInstance());
            const auto path2  = Astar::getRealPath(ugv_->getID(), ugv_->getPos(), common_point, NodeMap::getInstance());

            // 合并路径
            auto merged_path = path1;
            std::reverse(merged_path.begin(), merged_path.end());
            merged_path.insert(merged_path.end(), path2.begin(), path2.end());

            ROS_INFO("nodeset num: %lu", NodeMap::getInstance()->getNodeNum());

            Visualizer::getInstance(nh).visualizePath(merged_path);
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

    ros::Rate lr(1000);
    while (ros::ok()) {
        ros::spinOnce();
        lr.sleep();
    }
    return 0;
}
