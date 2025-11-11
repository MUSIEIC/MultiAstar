#include "visualizer.h"

#include <visualization_msgs/Marker.h>

Visualizer::Visualizer(ros::NodeHandle& nh) : nh_(nh) {
    spherePub_ = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
    pathPub_   = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
}

void Visualizer::visualizeStartGoal(const Eigen::Vector3d& center, const double& radius, int sg) {
    visualization_msgs::Marker sphereMarkers;

    sphereMarkers.id                 = sg;
    sphereMarkers.type               = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp       = ros::Time::now();
    sphereMarkers.header.frame_id    = "odom";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action             = visualization_msgs::Marker::ADD;
    sphereMarkers.ns                 = "StartGoal";
    sphereMarkers.color.r            = 1.00;
    sphereMarkers.color.g            = 0.00;
    sphereMarkers.color.b            = 0.00;
    sphereMarkers.color.a            = 1.00;
    sphereMarkers.scale.x            = radius * 2.0;
    sphereMarkers.scale.y            = radius * 2.0;
    sphereMarkers.scale.z            = radius * 2.0;

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

void Visualizer::visualizePath(const std::vector<Eigen::Vector3d>& route) {
    visualization_msgs::Marker routeMarker;

    routeMarker.id                 = 0;
    routeMarker.type               = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp       = ros::Time::now();
    routeMarker.header.frame_id    = "odom";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action             = visualization_msgs::Marker::ADD;
    routeMarker.ns                 = "route";
    routeMarker.color.r            = 1.00;
    routeMarker.color.g            = 0.00;
    routeMarker.color.b            = 0.00;
    routeMarker.color.a            = 1.00;
    routeMarker.scale.x            = 0.1;


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
