#include "Astar.h"
#include <chrono>

#include "Ros.hpp"

namespace RendezvousAstar {
    using AgentPtr = std::shared_ptr<Agent>;

    void plan(const Eigen::Vector3d& spos, const Eigen::Vector3d& epos, ros::NodeHandle& nh) {
        Astar astar;
        auto sposi     = NodeMap::posD2I(spos);
        auto eposi     = NodeMap::posD2I(epos);
        AgentPtr agent = std::make_shared<UAV>(1, spos, sposi);
        auto begin     = std::chrono::high_resolution_clock::now();
        auto state     = astar.run(agent, eposi, NodeMap::getInstance(), 1, {1}, [](const Astar::STATE& state) {
            return state == Astar::STATE::reached || state == Astar::STATE::reached_and_common;
        });
        auto end       = std::chrono::high_resolution_clock::now();

        ROS_INFO("PathSearch: node num: %lu ------ time: %ld ms", NodeMap::getInstance()->getNodeNum(),
            std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
        const auto path = Astar::getRealPath(1, sposi, eposi, NodeMap::getInstance());
        ROS_INFO("PathSearch: Astar state: %d,path size: %lu", state, path.size());
        Visualizer::getInstance(nh).visualizePath(path);
    }
} // namespace RendezvousAstar

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "standard_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    RendezvousAstar::PathSearch pathSearch(config, nh);
    pathSearch.setPlan([&nh](auto&& PH1, auto&& PH2) {
        RendezvousAstar::plan(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), nh);
    });

    ros::Rate lr(1000);
    while (ros::ok()) {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
