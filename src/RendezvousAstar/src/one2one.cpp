#include "Astar.h"
#include <future>
#include <queue>
#include <ros/ros.h>
#include <thread>

#include "Ros.hpp"
#include <one2one.hpp>

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "one2one_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    Visualizer::getInstance(nh);
    RendezvousAstar::PathSearch pathSearch(config, nh);

    bool multithread;
    ros::NodeHandle("~").getParam("multithreading", multithread);
    if (multithread) {
        pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
            RendezvousAstar::One2One o2o(s_pos[0], e_pos[0], nh);
            RendezvousAstar::Node::version_.fetch_add(1);
            Eigen::Vector3i rendezvous_node;
            o2o.planMultiThread(rendezvous_node);
            o2o.visualizePath(rendezvous_node);
        });
    } else {
        pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
            RendezvousAstar::One2One o2o(s_pos[0], e_pos[0], nh);
            RendezvousAstar::Node::version_.fetch_add(1);
            Eigen::Vector3i rendezvous_node;
            o2o.plan(rendezvous_node);
            o2o.visualizePath(rendezvous_node);
        });
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
