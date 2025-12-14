//
// Created by haung on 2025/11/11.
//

#ifndef MULTIASTAR_ROS_H
#define MULTIASTAR_ROS_H
#include "NodeMap.h"
#include "visualizer.h"
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <utility>
#include <vector>

namespace RendezvousAstar {
    class Agent;
    struct Config {
        std::string mapTopic;
        std::string targetTopic;
        double dilateRadius{};
        double voxelWidth{};
        std::vector<double> mapBound;
        bool initMapFirst{};

        explicit Config(const ros::NodeHandle& nh_) {
            nh_.getParam("MapTopic", mapTopic);
            nh_.getParam("InitMapFirst", initMapFirst);
            nh_.getParam("TargetTopic", targetTopic);
            nh_.getParam("DilateRadius", dilateRadius);
            nh_.getParam("VoxelWidth", voxelWidth);
            nh_.getParam("MapBound", mapBound);
        }
    };

    class PathSearch {
    public:
        PathSearch(Config conf, ros::NodeHandle& nh, const int32_t start_num = 1, const int32_t goal_num = 1)
            : config_(std::move(conf)), nh_(nh), mapOccupiedInited(false), start_nums_(start_num),
              goal_nums_(goal_num) {
            const Eigen::Vector3i xyz((config_.mapBound[1] - config_.mapBound[0]) / config_.voxelWidth,
                (config_.mapBound[3] - config_.mapBound[2]) / config_.voxelWidth,
                (config_.mapBound[5] - config_.mapBound[4]) / config_.voxelWidth);

            const Eigen::Vector3d offset(config_.mapBound[0], config_.mapBound[2], config_.mapBound[4]);

            NodeMap::initVoxelMap(xyz, offset, config_.voxelWidth);
            node_map_ = NodeMap::getInstance();

            mapSub_ =
                nh.subscribe(config_.mapTopic, 1, &PathSearch::mapCallBack, this, ros::TransportHints().tcpNoDelay());

            targetSub_ = nh.subscribe(
                config_.targetTopic, 1, &PathSearch::targetCallBack, this, ros::TransportHints().tcpNoDelay());

            pointSub_ =
                nh.subscribe("/initialpose", 1, &PathSearch::pointCallBack, this, ros::TransportHints().tcpNoDelay());
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
                    NodeMap::setOccupied(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
                }

                NodeMap::dilate(std::ceil(config_.dilateRadius / config_.voxelWidth));
                node_map_->preInit();

                mapOccupiedInited = true;
            }
        }

        void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            if (mapOccupiedInited) {
                if (start_goal_.size() >= start_nums_ + goal_nums_) {
                    start_goal_.clear();
                }
                const double zGoal = config_.mapBound[4] + config_.dilateRadius
                                   + fabs(msg->pose.orientation.z)
                                         * (config_.mapBound[5] - config_.mapBound[4] - 2 * config_.dilateRadius);
                const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
                // const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                if (NodeMap::query(goal) == 0) {
                    Visualizer::getInstance(nh_).visualizeStartGoal(goal, config_.voxelWidth, start_goal_.size(),start_nums_);
                    start_goal_.emplace_back(goal);
                } else {
                    ROS_WARN("Infeasible Position Selected !!!\n");
                }
                if (start_goal_.size() == start_nums_ + goal_nums_) {
                    plan_(std::vector(start_goal_.begin(), start_goal_.begin() + start_nums_),
                        std::vector(start_goal_.begin() + start_nums_, start_goal_.end()));
                }
            }
        }

        void pointCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) const {
            const auto point =
                Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            const Eigen::Vector3i p                        = NodeMap::posD2I(point);
            std::unordered_map<int32_t, std::string> state = {
                {0, "UNUSED"}, {1, "INOPEN"}, {2, "INCLOSE"}, {3, "INCOMMONSET"}};

            if (NodeMap::query(p)) {
                ROS_INFO("pointCallBack: 该位置为障碍物 \n");
                return;
            }

            const auto node = node_map_->getNode(p);
            if (!node) {
                ROS_INFO("pointCallBack: 该点未创建node \n");
                return;
            }
            ROS_INFO("pointCallBack: State: \n");
            for (const auto& s : node->getPathSet()) {
                ROS_INFO("pos: [%d,%d,%d] pathID: %d, State: %s, G: %.2f", node->getPos().x(), node->getPos().y(),
                    node->getPos().z(), s, state[node->getState(s)].c_str(), node->getG(s));
            }
        }

        void setPlan(
            std::function<void(const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos)>
                plan) {
            plan_ = std::move(plan);
        }

        void setStartPointNum(const int32_t start_num) {
            start_nums_ = start_num;
        }

        void setGoalPointNum(const int32_t goal_num) {
            goal_nums_ = goal_num;
        }

    private:
        Config config_;
        ros::NodeHandle nh_;
        ros::Subscriber mapSub_;
        ros::Subscriber targetSub_;
        ros::Subscriber pointSub_;
        std::shared_ptr<NodeMap> node_map_;
        std::vector<Eigen::Vector3d> start_goal_;
        std::function<void(const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos)> plan_;
        bool mapOccupiedInited;
        int32_t start_nums_, goal_nums_;
    };
} // namespace RendezvousAstar

#endif // MULTIASTAR_ROS_H
