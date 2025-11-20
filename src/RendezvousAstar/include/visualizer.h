#ifndef MULTIASTAR_VISUALIZER_H
#define MULTIASTAR_VISUALIZER_H

#include "Astar.h"
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <unordered_set>

namespace RendezvousAstar {
    struct Config;
}
class Visualizer {
private:
    ros::NodeHandle nh_;

    ros::Publisher spherePub_;
    ros::Publisher pathPub_;
    ros::Publisher commonPub_;
    ros::Publisher commonSetPub_;
    explicit Visualizer(ros::NodeHandle& nh);
    std::shared_ptr<RendezvousAstar::Config> config_;

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
    void visualizeStartGoal(const Eigen::Vector3d& center, const double& radius, int sg) const;
    void visualizePath(const std::vector<Eigen::Vector3d>& route) const;
    void visualizeCommon(const Eigen::Vector3d& common) const;
    void visualizeCommonset(
        const std::vector<std::shared_ptr<RendezvousAstar::Node>>& commonset) const;
};

#endif // MULTIASTAR_VISUALIZER_H
