//
// Created by haung on 2025/11/11.
//

#ifndef MULTIASTAR_ROS_H
#define MULTIASTAR_ROS_H
#include "NodeMap.h"
#include <chrono>
#include "visualizer.h"
#include <geometry_msgs/PoseStamped.h>
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

        explicit Config(const ros::NodeHandle& nh_) {
            nh_.getParam("MapTopic", mapTopic);
            nh_.getParam("TargetTopic", targetTopic);
            nh_.getParam("DilateRadius", dilateRadius);
            nh_.getParam("VoxelWidth", voxelWidth);
            nh_.getParam("MapBound", mapBound);
        }
    };

    class PathSearch {
    public:
        PathSearch(Config conf, ros::NodeHandle& nh) : config_(std::move(conf)), nh_(nh), mapOccupiedInited(false) {
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
                    NodeMap::getVoxelMap()->setOccupied(
                        Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
                }

                NodeMap::getVoxelMap()->dilate(std::ceil(config_.dilateRadius / NodeMap::getVoxelMap()->getScale()));

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
                if (NodeMap::getVoxelMap()->query(goal) == 0) {
                    Visualizer::getInstance(nh_).visualizeStartGoal(goal, 0.25, startgoal_.size());
                    startgoal_.emplace_back(goal);
                } else {
                    ROS_WARN("Infeasible Position Selected !!!\n");
                }
                if (startgoal_.size() == 2) {
                    plan_(startgoal_[0], startgoal_[1]);
                }
            }
        }

        void setPlan(std::function<void(const Eigen::Vector3d& spos, const Eigen::Vector3d& epos)> plan) {
            plan_ = std::move(plan);
        }

    private:
        Config config_;
        ros::NodeHandle nh_;
        ros::Subscriber mapSub;
        ros::Subscriber targetSub;
        std::shared_ptr<NodeMap> node_map_;
        std::vector<Eigen::Vector3d> startgoal_;
        std::function<void(const Eigen::Vector3d& spos, const Eigen::Vector3d& epos)> plan_;
        bool mapOccupiedInited;
    };
} // namespace RendezvousAstar

#endif // MULTIASTAR_ROS_H
