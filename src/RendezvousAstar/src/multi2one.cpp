#include "Astar.h"
#include <algorithm>
#include <atomic>
#include <future>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <thread>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "Ros.hpp"
#include "one2one.hpp"
#include <boost/smart_ptr/intrusive_ptr.hpp>

namespace RendezvousAstar {

    using AgentPtr = std::shared_ptr<Agent>;
    using NodePtr  = std::shared_ptr<Node>;
    class Multi2One {
    public:
        Multi2One(const std::vector<Eigen::Vector3d>& uav_pos, const std::vector<Eigen::Vector3d>& ugv_pos,
            const ros::NodeHandle& nh)
            : nh_(nh), uav_num_(uav_pos.size()), ugv_num_(ugv_pos.size()), astar_(std::make_shared<Astar>()),
              use_more_directs_(true), visualizer_(Visualizer::getInstance(nh_)),use_yaml_(false) {
            ros::NodeHandle("~").getParam("use_more_directs", use_more_directs_);
            ros::NodeHandle("~").getParam("use_yaml", use_yaml_);
            Eigen::Vector3i sum = {0, 0, 0};
            createUAVbyYaml();
            int j = 0;
            for (const auto& uav : uavs_) {
                if (!use_yaml_) {
                    uav->setPos(NodeMap::posD2I(uav_pos[j]));
                } else {
                    visualizer_.visualizeStartGoal(uav->getInitialPos(), 0.25, j, uavs_.size());
                }
                ++j;
                path_id_set_.push_back(uav->getID());
                uav_states_[uav->getID()] = Astar::STATE::searching;
                G_before_[uav->getID()]   = 0;
                sum += uav->getPos();
            }
            for (int32_t i = 0; i < ugv_pos.size(); ++i) {
                if (use_yaml_) {
                    visualizer_.visualizeStartGoal(ugv_pos[i], 0.25, j, uavs_.size());
                }
                ugvs_.emplace_back(std::make_shared<UGV>(-(i + 1), ugv_pos[i],0.2));
                ugv_states_[ugvs_[i]->getID()] = Astar::STATE::searching;
            }
            center_    = sum / uav_num_;
            center_[2] = std::static_pointer_cast<UGV>(ugvs_[0])->getHigh();
            ROS_INFO("Center: %d %d", center_[0], center_[1]);

            route_pub_    = nh_.advertise<geometry_msgs::PoseArray>("MultiAstarRoute", 10);
            priority_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("UAVPriorityQueue", 10);
        }
        ~Multi2One() = default;

        bool rendezvousCheck(const Eigen::Vector3i& point) const {
            auto node_map = NodeMap::getInstance();
            for (const auto& d : Astar::direct2d8_) {
                Eigen::Vector3i next_point = {
                    point[0] + static_cast<int32_t>(d[0]), point[1] + static_cast<int32_t>(d[1]), point[2]};
                const auto node = node_map->getNode(next_point);
                if (!NodeMap::query(next_point)) {
                    if (!node || node->getState(path_id_set_[0]) != Node::STATE::INCOMMONSET) {
                        return false;
                    }
                }
            }
            return true;
        }

        void createUAVbyYaml() {
            YAML::Node config      = YAML::LoadFile("/home/haung/prog/MultiAstar/src/RendezvousAstar/launch/uav.yaml");
            const YAML::Node& UAVs = config["UAVs"];
            int cnt                = 0;
            if (use_yaml_) {
                uav_num_=UAVs.size();
            }
            for (const auto& uav : UAVs) {
                if (++cnt > uav_num_) {
                    break;
                }
                auto id = uav["id"].as<int32_t>();
                Eigen::Vector3d init_pos;
                const YAML::Node& pos_node = uav["initialPosition"];
                int i                      = 0;
                for (const auto& pos : pos_node) {
                    init_pos[i++] = pos.as<double>();
                }
                auto power = uav["power"].as<double>();
                uavs_.emplace_back(std::make_shared<UAV>(id, init_pos, power));
            }
            if (cnt < uav_num_) {
                for (int i = 0; i < uav_num_ - cnt; ++i) {
                    uavs_.emplace_back(std::make_shared<UAV>(cnt + 1));
                }
            }
        }

        static double alphaG(double G, double alpha = 0.4, double a = 0.5, double G0 = 20.0) {
            double s = 1.0 / (1.0 + std::exp(-a * (G - G0)));
            return alpha + (1.0 - alpha) * s;
        }

        static double N_G(double G, double lam = 0.1) {
            return 1.0 - std::exp(-lam * G);
        }

        static double score(const NodePtr& node, const AgentPtr& agent) {
            double p1  = agent->getPower();
            double np1 = (p1 - 0.2) / 0.8;
            double ng1 = N_G(node->getG(agent->getID()), 0.1);

            double ag1 = alphaG(node->getG(agent->getID()), 0.4, 0.5, 5.0);
            double s1  = (1.0 - ag1) * (1.0 - ng1) + ag1 * (1.0 - np1);
            s1         = p1 >= 0.2 ? s1 : 1.0;
            return s1;
        }

        static void computePriority(const NodePtr& node, std::vector<AgentPtr>& UAVs) {

            std::sort(UAVs.begin(), UAVs.end(), [&node](const AgentPtr& uav1, const AgentPtr& uav2) {
                const double s1 = score(node, uav1);
                const double s2 = score(node, uav2);
                return s1 > s2;
            });
        }

        double computeCost(const NodePtr& node) const {
            double t_g      = node->getG(ugvs_[0]->getID()) / 0.3; // ugv speed :0.3
            double cost     = 0.0;
            double t_l      = 1.0;
            double t_before = -1000;
            for (const auto& uav : uavs_) {
                double t = node->getG(uav->getID()) / (0.5 + (0.2 * uav->getPower()));
                t        = std::max({t, t_before + t_l, t_g});
                cost     = std::max(cost, t + t_l);
                t_before = t;
            }

            return cost;
        }

        bool compare(const NodePtr& n1, const NodePtr& n2) const {

            auto UAVs = uavs_;

            computePriority(n1, UAVs);
            double cost1 = computeCost(n1);

            computePriority(n2, UAVs);
            double cost2 = computeCost(n2);

            return cost1 < cost2;
        }

        void plan() {
            std::promise<NodePtr> rendezvous_promise;
            std::future<NodePtr> rendezvous_future = rendezvous_promise.get_future();
            bool getRendezvous{false};

            stop_.store(false);
            std::vector<std::thread> threads;

            const auto time_begin = std::chrono::high_resolution_clock::now();

            for (auto& ugv : ugvs_) {
                auto path_id_set = path_id_set_;
                path_id_set.emplace_back(ugv->getID());
                threads.emplace_back([this, &ugv, path_id_set]() {
                    const auto state = astar_->run(
                        ugv, ugv, center_, ugv->getID(), path_id_set,
                        [this](const Astar::STATE& sat) {
                            return stop_.load() || sat == Astar::STATE::map_search_done
                                || sat == Astar::STATE::over_time;
                        },
                        use_more_directs_);
                    {
                        std::unique_lock lock(mutex_);
                        ugv_states_[ugv->getID()] = state;
                    }
                    ROS_INFO("UGV id: %d  State: %d", ugv->getID(), state);
                });
            }


            for (auto& uav : uavs_) {
                auto path_id_set = path_id_set_;
                path_id_set.emplace_back(ugvs_[0]->getID());
                threads.emplace_back([this, &uav, path_id_set]() {
                    const auto state = astar_->run(
                        uav, ugvs_[0], center_, uav->getID(), path_id_set,
                        [this](const Astar::STATE& sat) {
                            return stop_.load() || sat == Astar::STATE::map_search_done
                                || sat == Astar::STATE::over_time;
                        },
                        use_more_directs_);
                    // if (state == Astar::STATE::map_search_done || state == Astar::STATE::over_time) {
                    //     stop_.store(true);
                    // }
                    uav_count_.fetch_add(1);
                    if (uav_count_.load() == uavs_.size()) {
                        stop_.store(true);
                    }
                    ROS_INFO("UAV id: %d  State: %d", uav->getID(), state);
                });
            }


            threads.emplace_back([this, &rendezvous_promise, &getRendezvous]() {
                while (!stop_.load()) {
                    std::vector<NodePtr> snapshot = astar_->getCommonSet();
                    if (!snapshot.empty()) {
                        auto id_set = path_id_set_;
                        id_set.emplace_back(ugvs_[0]->getID());
                        sort(snapshot.begin(), snapshot.end(),
                            [this](const NodePtr& n1, const NodePtr& n2) { return compare(n1, n2); });

                        const auto node = snapshot.front();
                        if (rendezvousCheck(node->getPos())) {
                            rendezvous_promise.set_value(node);
                            getRendezvous = true;
                            stop_.store(true);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            });

            for (auto& thread : threads) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
            visualizer_.visualizeCommonSet(astar_->getCommonSet());


            NodePtr rendezvous_node;
            const auto time_end = std::chrono::high_resolution_clock::now();
            ROS_INFO("commonSet size: %lu", astar_->getCommonNum());
            ROS_INFO("multi2one time: %ld",
                std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() / 1000);

            if (getRendezvous) {
                try {
                    ROS_INFO("获取汇合点");
                    rendezvous_node = rendezvous_future.get();
                    for (const auto& uav : uavs_) {
                        ROS_INFO("UAV id: %d  Priority: %f", uav->getID(), score(rendezvous_node, uav));
                    }
                } catch (const std::future_error& e) {
                    ROS_WARN("路径生成失败，未得到汇合点 %s", e.what());
                    return;
                }
            } else {
                ROS_WARN("路径生成失败，未得到汇合点");
                return;
            }


            visualizer_.visualizeCommon(NodeMap::posI2D(rendezvous_node->getPos()));
            std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> paths;
            for (const auto& uav : uavs_) {
                auto path =
                    Astar::getRealPath(uav->getID(), uav->getPos(), rendezvous_node->getPos(), NodeMap::getInstance());
                std::reverse(path.begin(),path.end());
                path[0]=uav->getInitialPos();
                path.back()=NodeMap::posI2D(rendezvous_node->getPos());
                paths[uav->getID()]=path;
                visualizer_.visualizePath(path, uav->getID());
            }

            const auto ugv_path = Astar::getRealPath(
                ugvs_[0]->getID(), ugvs_[0]->getPos(), rendezvous_node->getPos(), NodeMap::getInstance());
                paths[ugvs_[0]->getID()]=ugv_path;
            visualizer_.visualizePath(ugv_path, ugvs_[0]->getID());

            for (const auto& path : paths) {
                routePub(path.second, path.first);
                ros::Duration(0.2).sleep();
            }
            auto uavss = uavs_;
            computePriority(rendezvous_node, uavss);
            std_msgs::Int32MultiArray msg;
            for (const auto& uav : uavss) {
                msg.data.push_back(uav->getID());
            }
            ROS_INFO("pub Priority");
            priority_pub_.publish(msg);
            ros::Duration(0.2).sleep();
        }

        static bool endCondition(const Astar::STATE& state) {
            return state == Astar::STATE::common_over_threshold || state == Astar::STATE::reached_and_common;
        }

        void routePub(const std::vector<Eigen::Vector3d>& route, const int32_t id) const {
            geometry_msgs::PoseArray pose_array;
            for (const auto& point : route) {
                geometry_msgs::Pose pose;
                pose.position.x    = point[0];
                pose.position.y    = point[1];
                pose.position.z    = point[2];
                pose.orientation.x = id;
                pose_array.poses.push_back(pose);
            }
            route_pub_.publish(pose_array);
        }


    private:
        ros::NodeHandle nh_;
        std::vector<AgentPtr> uavs_, ugvs_;
        int32_t uav_num_, ugv_num_;
        std::unordered_map<int32_t, Astar::STATE> uav_states_, ugv_states_;
        std::unordered_map<int32_t, double> G_before_;
        std::vector<int32_t> path_id_set_;
        std::atomic<bool> stop_{false};
        std::atomic<int> uav_count_{0};
        std::shared_mutex mutex_;
        std::shared_ptr<Astar> astar_;
        bool use_more_directs_;
        Eigen::Vector3i center_;
        Visualizer& visualizer_;
        std::shared_ptr<NodeMap> node_map_ = NodeMap::getInstance();
        ros::Publisher route_pub_;
        ros::Publisher priority_pub_;
        bool use_yaml_;
    };

} // namespace RendezvousAstar


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi2one_searching_node");
    ros::NodeHandle nh;
    RendezvousAstar::Config config(ros::NodeHandle("~"));
    Visualizer::getInstance(nh);
    int uav_num = 1, ugv_num = 1;
    ros::NodeHandle("~").getParam("uav_num", uav_num);
    ros::NodeHandle("~").getParam("ugv_num", ugv_num);
    RendezvousAstar::PathSearch pathSearch(config, nh, uav_num, ugv_num);

    pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
        RendezvousAstar::Node::version_.fetch_add(1);
        RendezvousAstar::Multi2One multi2one(s_pos, e_pos, nh);
        multi2one.plan();
    });

    // //-----------------------test-----------------------------
    // pathSearch.setPlan([&nh](const std::vector<Eigen::Vector3d>& s_pos, const std::vector<Eigen::Vector3d>& e_pos) {
    //     std::vector<Eigen::Vector3d> s_pos_t = {
    //         {-13.447, 17.039, 1.0},
    //         {-6.527, 9.291, 1.0},
    //         {-0.891, 20.033, 1.0},
    //         {0.313, 1.046, 1.0},
    //         {18.125, 7.565, 1.0},
    //         {-18.541, 3.072, 1.0},
    //         {-19.353, -1.147, 1.0},
    //         {4.871, 16.593, 1.0},
    //         {0.439, 6.427, 1.0},
    //         {-5.866, -2.508, 1.0},
    //         {11.612, -7.348, 1.0},
    //     };
    //
    //     std::vector<Eigen::Vector3d> e_pos_t = {{9.758, -10.795, 1.0}};
    //     RendezvousAstar::Node::version_.fetch_add(1);
    //     RendezvousAstar::Multi2One multi2one(s_pos_t, e_pos_t, nh);
    //     multi2one.plan();
    // });
    //--------------------------------------------------------
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
