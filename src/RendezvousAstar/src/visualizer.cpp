#include "visualizer.h"

#include <Eigen/Eigen>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <Ros.hpp>

Visualizer::Visualizer(ros::NodeHandle& nh)
    : nh_(nh), config_(std::make_shared<RendezvousAstar::Config>(ros::NodeHandle("~"))) {
    spherePub_    = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
    pathPub_      = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
    commonPub_    = nh.advertise<geometry_msgs::PointStamped>("/visualizer/common_point", 10);
    commonSetPub_ = nh.advertise<visualization_msgs::Marker>("/visualizer/common_set", 1000);
}

void Visualizer::visualizeStartGoal(
    const Eigen::Vector3d& center, const double& radius, int sg, int change_color) const {
    visualization_msgs::Marker sphereMarkers;

    sphereMarkers.id                 = sg;
    sphereMarkers.type               = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp       = ros::Time::now();
    sphereMarkers.header.frame_id    = "odom";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action             = visualization_msgs::Marker::ADD;
    sphereMarkers.ns                 = "StartGoal";
    if (sg == change_color) {
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
    } else {
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
    }
    sphereMarkers.color.a                    = 1.00;
    sphereMarkers.scale.x                    = radius / 2;
    sphereMarkers.scale.y                    = radius / 2;
    sphereMarkers.scale.z                    = radius / 2;
    visualization_msgs::Marker sphereDeleter = sphereMarkers;
    sphereDeleter.action                     = visualization_msgs::Marker::DELETEALL;

    geometry_msgs::Point point;
    point.x = center(0);
    point.y = center(1);
    point.z = center(2);
    sphereMarkers.points.push_back(point);

    if (sg == 0) {
        spherePub_.publish(sphereDeleter);
        ros::Duration(1.0e-9).sleep();
        sphereMarkers.header.stamp = ros::Time::now();
    }
    spherePub_.publish(sphereMarkers);
}

void Visualizer::visualizePath(const std::vector<Eigen::Vector3d>& route, const int32_t& id) const {
    visualization_msgs::Marker routeMarker;

    routeMarker.id                 = id;
    routeMarker.type               = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp       = ros::Time::now();
    routeMarker.header.frame_id    = "odom";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action             = visualization_msgs::Marker::ADD;
    routeMarker.ns                 = "route";
    if (id > 0) {
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
    } else {
        routeMarker.color.r = 0.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 1.00;
    }
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.3;
    routeMarker.scale.y = 0.3;
    routeMarker.scale.z = 0.3;

    if (route.size() > 0) {
        bool first = true;
        Eigen::Vector3d last;
        for (auto it : route) {
            if (first) {
                first = false;
                last  = it;
                continue;
            }
            geometry_msgs::Point point;

            point.x = last(0);
            point.y = last(1);
            point.z = last(2);
            routeMarker.points.push_back(point);
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            routeMarker.points.push_back(point);
            last = it;
        }

        pathPub_.publish(routeMarker);
    }
}

// 移除了inline关键字，确保函数正确链接
void Visualizer::visualizeCommon(const Eigen::Vector3d& common) const {
    geometry_msgs::PointStamped p;
    p.header.frame_id = "odom";
    p.header.stamp    = ros::Time::now();
    p.point.x         = common.x();
    p.point.y         = common.y();
    p.point.z         = common.z();
    commonPub_.publish(p);
}
void Visualizer::visualizeCommonSet(const std::vector<std::shared_ptr<RendezvousAstar::Node>>& common_set) const {
    visualization_msgs::Marker comarker, comarker2;
    geometry_msgs::Point p;
    comarker.header.frame_id = "odom";
    comarker.ns              = "common_set";
    comarker.id              = 0;
    comarker.type            = visualization_msgs::Marker::CUBE_LIST;
    comarker.action          = visualization_msgs::Marker::ADD;
    comarker.scale.x         = config_->voxelWidth;
    comarker.scale.y         = config_->voxelWidth;
    comarker.scale.z         = config_->voxelWidth;
    comarker.color.r         = 233;
    comarker.color.g         = 185;
    comarker.color.b         = 110;
    comarker.color.a         = 0.5;

    for (const auto& s : common_set) {
        auto rs = RendezvousAstar::NodeMap::posI2D(s->getPos());
        p.x     = rs.x();
        p.y     = rs.y();
        p.z     = rs.z();
        comarker.points.push_back(p);
    }
    commonSetPub_.publish(comarker);
}

void Visualizer::visualizeCommonSet(const std::vector<std::shared_ptr<RendezvousAstar::Node>>& common_set,
    const std::vector<std::shared_ptr<RendezvousAstar::Node>>& nodes) const {
    visualization_msgs::Marker comarker, comarker2;
    geometry_msgs::Point p;
    comarker.header.frame_id = "odom";
    comarker.ns              = "common_set";
    comarker.id              = 0;
    comarker.type            = visualization_msgs::Marker::CUBE_LIST;
    comarker.action          = visualization_msgs::Marker::ADD;
    comarker.scale.x         = config_->voxelWidth;
    comarker.scale.y         = config_->voxelWidth;
    comarker.scale.z         = config_->voxelWidth;
    comarker.color.r         = 233;
    comarker.color.g         = 185;
    comarker.color.b         = 110;
    comarker.color.a         = 0.4;

    comarker2         = comarker;
    comarker2.id      = 1;
    comarker2.color.a = 0.8;
    for (const auto& s : common_set) {
        auto rs = RendezvousAstar::NodeMap::posI2D(s->getPos());
        p.x     = rs.x();
        p.y     = rs.y();
        p.z     = rs.z();
        comarker.points.push_back(p);
    }
    for (const auto &s : nodes) {
        auto rs = RendezvousAstar::NodeMap::posI2D(s->getPos());
        p.x = rs.x();
        p.y = rs.y();
        p.z = rs.z();
        comarker2.points.push_back(p);
    }
    commonSetPub_.publish(comarker);
    commonSetPub_.publish(comarker2);
}
