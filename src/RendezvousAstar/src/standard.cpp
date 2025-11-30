#include "Astar.h"
#include <chrono>

#include "Ros.hpp"

namespace RendezvousAstar {
    using AgentPtr = std::shared_ptr<Agent>;

    void plan(const Eigen::Vector3d& s_pos, const Eigen::Vector3d& e_pos, ros::NodeHandle& nh) {
        Node::version_.fetch_add(1);
        Astar astar;
        AgentPtr agent   = std::make_shared<UAV>(1, s_pos);
        AgentPtr e_agent = std::make_shared<UGV>(-1, e_pos);
        auto begin       = std::chrono::high_resolution_clock::now();
        auto state       = astar.run(agent, e_agent, e_agent->getPos(), 1, {1}, [](const Astar::STATE& state) {
            return state == Astar::STATE::reached || state == Astar::STATE::reached_and_common;
        });
        auto end         = std::chrono::high_resolution_clock::now();

        ROS_INFO("PathSearch: node num: %lu ------ time: %ld ms", NodeMap::getInstance()->getNodeNum(),
            std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
        const auto path = Astar::getRealPath(1, agent->getPos(), e_agent->getPos(), NodeMap::getInstance());
        ROS_INFO("PathSearch: Astar state: %d,path size: %lu", state, path.size());
        Visualizer::getInstance(nh).visualizePath(path, 0);
        // Visualizer::getInstance(nh).visualizeCommonSet(astar.getCommonSet());
    }
} // namespace RendezvousAstar

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "standard_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    RendezvousAstar::PathSearch pathSearch(config, nh);
    pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
        RendezvousAstar::plan(s_pos[0], e_pos[0], nh);
    });

    ros::Rate lr(1000);
    while (ros::ok()) {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
