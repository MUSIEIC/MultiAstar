#ifndef MULTIASTAR_VISUALIZER_H
#define MULTIASTAR_VISUALIZER_H

#include <Eigen/Eigen>
#include <ros/ros.h>
class Visualizer {
private:
    ros::NodeHandle nh_;

    ros::Publisher spherePub_;
    ros::Publisher pathPub_;

public:
    Visualizer(ros::NodeHandle& nh);
    ~Visualizer() = default;

    void visualizeStartGoal(const Eigen::Vector3d& center, const double& radius, int sg);
    void visualizePath(const std::vector<Eigen::Vector3d>& route);
};

#endif // MULTIASTAR_VISUALIZER_H
