#ifndef MULTIASTAR_VISUALIZER_H
#define MULTIASTAR_VISUALIZER_H

#include <Eigen/Eigen>
#include <ros/ros.h>
class Visualizer {
private:
    ros::NodeHandle nh_;

    ros::Publisher spherePub_;
    ros::Publisher pathPub_;
    Visualizer(ros::NodeHandle& nh);

public:
    ~Visualizer()                            = default;
    Visualizer(const Visualizer&)            = delete;
    Visualizer& operator=(const Visualizer&) = delete;
    Visualizer(Visualizer&&)                 = delete;
    Visualizer& operator=(Visualizer&&)      = delete;

    static Visualizer& getInstance(ros::NodeHandle& nh) {
        static Visualizer instance(nh);
        return instance;
    }
    void visualizeStartGoal(const Eigen::Vector3d& center, const double& radius, int sg);
    void visualizePath(const std::vector<Eigen::Vector3d>& route);
};

#endif // MULTIASTAR_VISUALIZER_H
