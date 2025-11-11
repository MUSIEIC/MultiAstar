#include "Astar.h"
#include "NodeMap.h"
#include "visualizer.h"
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

namespace RendezvousAstar {
    struct Config {
        std::string mapTopic;
        std::string targetTopic;
        double dilateRadius;
        double voxelWidth;
        std::vector<double> mapBound;

        Config(const ros::NodeHandle& nh_) {
            nh_.getParam("MapTopic", mapTopic);
            nh_.getParam("TargetTopic", targetTopic);
            nh_.getParam("DilateRadius", dilateRadius);
            nh_.getParam("VoxelWidth", voxelWidth);
            nh_.getParam("MapBound", mapBound);
        }
    };

    class PathSearch {
    public:
        PathSearch(const Config& conf, ros::NodeHandle& nh)
            : config_(conf), nh_(nh), mapOccupiedInited(false), visualizer_(nh) {
            const Eigen::Vector3i xyz((config_.mapBound[1] - config_.mapBound[0]) / config_.voxelWidth,
                (config_.mapBound[3] - config_.mapBound[2]) / config_.voxelWidth,
                (config_.mapBound[5] - config_.mapBound[4]) / config_.voxelWidth);

            const Eigen::Vector3d offset(config_.mapBound[0], config_.mapBound[2], config_.mapBound[4]);

            NodeMap::initVoxelMap(xyz, offset, config_.voxelWidth);
            node_map_ = NodeMap::getInstance();

            mapSub =
                nh.subscribe(config_.mapTopic, 1, &PathSearch::mapCallBack, this, ros::TransportHints().tcpNoDelay());

            targetSub = nh.subscribe(
                config_.targetTopic, 1, &PathSearch::targetCallBack, this, ros::TransportHints().tcpNoDelay());
        }

        void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            if (!mapOccupiedInited) {
                size_t cur         = 0;
                const size_t total = msg->data.size() / msg->point_step;
                float* fdata       = (float*) (&msg->data[0]);
                for (size_t i = 0; i < total; i++) {
                    cur = msg->point_step / sizeof(float) * i;

                    if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) || std::isnan(fdata[cur + 1])
                        || std::isinf(fdata[cur + 1]) || std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
                        continue;
                    }
                    node_map_->getVoxelMap()->setOccupied(
                        Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
                }

                node_map_->getVoxelMap()->dilate(
                    std::ceil(config_.dilateRadius / node_map_->getVoxelMap()->getScale()));

                mapOccupiedInited = true;
            }
        }

        void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            if (mapOccupiedInited) {
                if (startgoal_.size() >= 2) {
                    startgoal_.clear();
                }
                const double zGoal = config_.mapBound[4] + config_.dilateRadius
                                   + fabs(msg->pose.orientation.z)
                                         * (config_.mapBound[5] - config_.mapBound[4] - 2 * config_.dilateRadius);
                const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
                // const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                if (node_map_->getVoxelMap()->query(goal) == 0) {
                    visualizer_.visualizeStartGoal(goal, 0.25, startgoal_.size());
                    startgoal_.emplace_back(goal);
                } else {
                    ROS_WARN("Infeasible Position Selected !!!\n");
                }
                if (startgoal_.size() == 2) {
                    Astar astar;
                    auto spos                    = node_map_->posD2I(startgoal_[0]);
                    auto gpos                    = node_map_->posD2I(startgoal_[1]);
                    std::shared_ptr<Agent> agent = std::make_shared<UAV>(node_map_, 1, startgoal_[0], spos);
                    // auto node                    = node_map_->getNode(spos);
                    // if (!node) {
                    //     node = std::make_shared<Node>(1, nullptr, agent->getPos());
                    // }
                    // node->setG(1, 0.0);
                    // node_map_->addNode(node);
                    auto begin = std::chrono::high_resolution_clock::now();
                    auto state = astar.run(agent, gpos, node_map_, 1, 1, [](const Astar::STATE& state) {
                        return state == Astar::STATE::reached || state == Astar::STATE::reached_and_common;
                    });
                    auto end   = std::chrono::high_resolution_clock::now();
                    ROS_INFO("PathSearch: node num: %lu ------ time: %ld ms", node_map_->getNodeNum(),
                        std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
                    auto path = Astar::getRealPath(1, spos, gpos, node_map_);
                    ROS_INFO("PathSearch: Astar state: %d,path size: %lu", state, path.size());

                    visualizer_.visualizePath(path);
                }
            }
        }

    private:
        Config config_;
        ros::NodeHandle nh_;
        ros::Subscriber mapSub;
        ros::Subscriber targetSub;
        std::shared_ptr<NodeMap> node_map_;
        std::vector<Eigen::Vector3d> startgoal_;
        bool mapOccupiedInited;
        Visualizer visualizer_;
    };

} // namespace RendezvousAstar

int main(int argc, char** argv) {

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "rendezvous_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));

    RendezvousAstar::PathSearch path_search(config, nh);


    ros::Rate lr(1000);
    while (ros::ok()) {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
