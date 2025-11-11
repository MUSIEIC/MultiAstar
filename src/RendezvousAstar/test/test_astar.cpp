#include "Agent.h"
#include "Astar.h"
#include "NodeMap.h"
#include "node.h"
#include <array>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <unordered_set>

using namespace RendezvousAstar;

// // 测试Astar类构造函数和基本访问器
// TEST(AstarTest, ConstructorAndBasicAccess)
// {
//     // 测试默认构造函数
//     Astar astar1;
//
//     // 测试带参数的构造函数
//     Astar astar2(10, 15);
//
//     // 构造后对象应能正常使用
//     Eigen::Vector3i start(0, 0, 0);
//     Eigen::Vector3i end(3, 4, 0);
//     double distance = Astar::computeH(start, end);
//     EXPECT_DOUBLE_EQ(distance, 5.0);
// }
//
// // 测试Astar类的基本功能
// TEST(AstarTest, ComputeHFunction)
// {
//     // 测试欧几里得距离计算
//     Eigen::Vector3i start(0, 0, 0);
//     Eigen::Vector3i end(3, 4, 0);
//
//     double distance = Astar::computeH(start, end, "euclidean");
//     EXPECT_DOUBLE_EQ(distance, 5.0); // 3-4-5直角三角形斜边长度
//
//     // 测试曼哈顿距离计算
//     double manhattan_distance = Astar::computeH(start, end, "manhattan");
//     EXPECT_DOUBLE_EQ(manhattan_distance, 7.0); // |3-0| + |4-0| + |0-0| = 7
//
//     // 测试默认方法（欧几里得）
//     double default_distance = Astar::computeH(start, end);
//     EXPECT_DOUBLE_EQ(default_distance, 5.0);
// }
//
// // 测试Astar运行一次的状态返回
// TEST(AstarTest, RunOnceBasic)
// {
//     // 创建Astar实例
//     Astar astar;
//
//     // 创建Agent
//     std::shared_ptr<Agent> agent = std::make_shared<UAV>();
//
//     // 获取NodeMap实例
//     auto nodeMap = NodeMap::getInstance();
//
//     // 设置目标点
//     Eigen::Vector3i target(5, 5, 5);
//
//     // 初始化路径ID和路径数量
//     int32_t path_id = 1;
//     int32_t path_num = 2;
//
//     // 第一次运行应该返回map_search_done状态，因为open_list为空
//     Astar::STATE state = astar.runOnce(agent, target, nodeMap, path_id, path_num);
//     EXPECT_EQ(state, Astar::STATE::map_search_done);
// }

// 测试Astar完整的路径搜索过程
TEST(AstarTest, FullPathSearch)
{
    // 创建Astar实例
    Astar astar;
    
    // 创建Agent
    std::shared_ptr<Agent> agent = std::make_shared<UAV>();
    
    // 获取NodeMap实例
    auto nodeMap = NodeMap::getInstance();
    
    // 创建起始节点并添加到open_list
    auto startNode = std::make_shared<Node>(1, nullptr, 0, 0, 0);
    nodeMap->addNode(startNode);
    startNode->setG(1,0.0);
    // 将起始节点添加到Agent的open_list中
    agent->getOpenList(1).insert(std::array<double,4>{0.0, 0.0, 0.0, 0.0});
    
    // 设置目标点
    Eigen::Vector3i target(0, 1,0);

    auto t=nodeMap->posI2D(target);
    ROS_WARN("real pos: %f,%f,%f",t.x(),t.y(),t.z());
    
    // 初始化路径ID和路径数量
    int32_t path_id = 1;
    int32_t path_num = 1;
    
    // 运行几次直到找到路径或确定无解
    // Astar::STATE state = Astar::STATE::searching;
    // int max_iterations = 100000; // 防止无限循环
    // int iterations = 0;
    //
    // auto start_time=std::chrono::high_resolution_clock::now();
    // while (state == Astar::STATE::searching && iterations < max_iterations) {
    //     state = astar.runOnce(agent, target, nodeMap, path_id, path_num);
    //     iterations++;
    //     if (iterations%1000==0)
    //     {
    //         ROS_WARN("iterations: %d",iterations);
    //     }
    // }
    // auto end_time=std::chrono::high_resolution_clock::now();
    // ROS_WARN("time: %f ms",std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count()/1000.0);
    auto state = astar.run(agent, target, nodeMap, path_id, path_num,
        [](const Astar::STATE &reachstate)
    {
        return reachstate==Astar::STATE::reached||reachstate==Astar::STATE::reached_and_common;
        return reachstate==Astar::STATE::common_over_threshold;
    });
    ROS_WARN("state: %d",static_cast<int32_t>(state));
    // 验证搜索最终结束
    EXPECT_NE(state, Astar::STATE::searching);

    auto path=Astar::getVoxelPath(path_id,target,nodeMap);

    for (auto &p:path)
    {
        ROS_WARN("x: %d, y: %d, z: %d",p.x(),p.y(),p.z());
    }
    ROS_WARN("size: %d",static_cast<int32_t>(nodeMap->getNodeNum()));
    ROS_WARN("common size: %d",static_cast<int32_t>(astar.getCommonNum()));
    // nodeMap->printNodeSet();
    // 状态应该是map_search_done（如果找不到路径）或者reached（如果找到路径）
    // EXPECT_TRUE(state == Astar::STATE::reached);
}

// // 测试Astar在2D环境下的行为（针对UGV）
// TEST(AstarTest, TwoDimensionalSearch)
// {
//     // 创建Astar实例
//     Astar astar;
//
//     // 创建UGV Agent（2D搜索）
//     std::shared_ptr<Agent> agent = std::make_shared<UGV>();
//
//     // 获取NodeMap实例
//     auto nodeMap = NodeMap::getInstance();
//
//     // 创建起始节点并添加到open_list
//     auto startNode = std::make_shared<Node>(1, nullptr, 0, 0, 0);
//     nodeMap->addNode(startNode);
//
//     // 将起始节点添加到Agent的open_list中
//     agent->getOpenList(1).push(std::array<double,4>{0.0, 0.0, 0.0, 0.0});
//
//     // 设置目标点
//     Eigen::Vector3i target(4, 2, 0);
//
//     // 初始化路径ID和路径数量
//     int32_t path_id = 1;
//     int32_t path_num = 2;
//
//     // 运行几次直到找到路径或确定无解
//     Astar::STATE state = Astar::STATE::searching;
//     int max_iterations = 50; // 防止无限循环
//     int iterations = 0;
//
//     while (state == Astar::STATE::searching && iterations < max_iterations) {
//         state = astar.runOnce(agent, target, nodeMap, path_id, path_num);
//         iterations++;
//     }
//
//     // 验证搜索最终结束
//     EXPECT_NE(state, Astar::STATE::searching);
//     auto tn=nodeMap->getNode(target);
//     while (tn)
//     {
//         ROS_WARN("pos x: %d, y: %d, z: %d",tn->getPos().x(), tn->getPos().y(), tn->getPos().z());
//         tn=tn->getParent(path_id);
//     }
//     // 状态应该是map_search_done（如果找不到路径）或者reached（如果找到路径）
//     EXPECT_TRUE(state == Astar::STATE::reached);
// }
//
// // 测试setThreshold方法
// TEST(AstarTest, SetThreshold)
// {
//     // 创建Astar实例
//     Astar astar(5, 10); // 默认阈值为5
//
//     // 测试setThreshold方法
//     astar.setThreshold(10);
//

//     // 因为common_set_是私有成员变量，不能直接访问
//     // 但我们至少验证对象可以正常构造和使用
//     Eigen::Vector3i start(0, 0, 0);
//     Eigen::Vector3i end(1, 1, 1);
//     double distance = Astar::computeH(start, end);
//     EXPECT_GT(distance, 0);
// }
//
// // 测试边界条件和特殊情况
// TEST(AstarTest, EdgeCases)
// {
//     // 创建Astar实例
//     Astar astar;
//
//     // 创建Agent
//     std::shared_ptr<Agent> agent = std::make_shared<UAV>();
//
//     // 获取NodeMap实例
//     auto nodeMap = NodeMap::getInstance();
//
//     // 测试相同起点和终点的情况
//     Eigen::Vector3i target(0, 0, 0);
//     int32_t path_id = 1;
//     int32_t path_num = 2;
//
//     // 创建起始节点并添加到open_list
//     auto startNode = std::make_shared<Node>(1, nullptr, 0, 0, 0);
//     nodeMap->addNode(startNode);
//
//     // 将起始节点添加到Agent的open_list中
//     agent->getOpenList(1).push(std::array<double,4>{0.0, 0.0, 0.0, 0.0});
//
//     // 运行一次应该立即到达目标
//     Astar::STATE state = astar.runOnce(agent, target, nodeMap, path_id, path_num);
//     // 根据当前实现，即使起点等于终点，也需要检查common_set_大小才能决定返回值
//     EXPECT_TRUE(state == Astar::STATE::reached );
// }
//
// // 测试多个Agent协同搜索的情况
// TEST(AstarTest, MultiAgentSearch)
// {
//     // 创建Astar实例
//     Astar astar;
//
//     // 创建两个Agent
//     std::shared_ptr<Agent> agent1 = std::make_shared<UAV>();
//     std::shared_ptr<Agent> agent2 = std::make_shared<UGV>();
//
//     // 获取NodeMap实例
//     auto nodeMap = NodeMap::getInstance();
//
//     // 设置共同目标点
//     Eigen::Vector3i target(3, 3, 0);
//
//     // 为两个Agent分别创建起始节点
//     auto startNode1 = std::make_shared<Node>(1, nullptr, 0, 0, 0);
//     auto startNode2 = std::make_shared<Node>(2, nullptr, 5, 5, 0);
//     nodeMap->addNode(startNode1);
//     nodeMap->addNode(startNode2);
//
//     // 将起始节点添加到各自Agent的open_list中
//     agent1->getOpenList(1).push(std::array<double,4>{0.0, 0.0, 0.0, 0.0});
//     agent2->getOpenList(2).push(std::array<double,4>{0.0, 5.0, 5.0, 0.0});
//
//     // 分别运行两个Agent的搜索
//     Astar::STATE state1 = astar.runOnce(agent1, target, nodeMap, 1, 2);
//     Astar::STATE state2 = astar.runOnce(agent2, target, nodeMap, 2, 2);
//
//     // 验证两个Agent都开始搜索
//     EXPECT_NE(state1, Astar::STATE::failed);
//     EXPECT_NE(state2, Astar::STATE::failed);
// }