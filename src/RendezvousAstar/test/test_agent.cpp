#include <gtest/gtest.h>
#include "Agent.h"

using namespace RendezvousAstar;

// 创建一个具体的Agent子类用于测试
class TestAgent : public Agent {
public:
    TestAgent() : Agent() {}
    TestAgent(int32_t id, const Eigen::Vector3d& initial_pos) : Agent(id, initial_pos) {}
    ~TestAgent() override = default;
    
    Queue& getOpenList(int32_t id) override {
        return open_list_;
    }
    
    List& getClosedList(int32_t id) override {
        return closed_list_;
    }

private:
    Queue open_list_;
    List closed_list_;
};

// 测试Agent基类
TEST(AgentTest, ConstructorAndBasicAccess)
{
    // 测试默认构造函数
    TestAgent agent;
    EXPECT_EQ(agent.getID(), 0);
    EXPECT_EQ(agent.getInitialPos(), Eigen::Vector3d::Zero());

    // 测试带参数的构造函数
    Eigen::Vector3d initial_pos(1.0, 2.0, 3.0);
    TestAgent agent2(5, initial_pos);
    EXPECT_EQ(agent2.getID(), 5);
    EXPECT_EQ(agent2.getInitialPos(), initial_pos);
}

TEST(AgentTest, SetAndGetMethods)
{
    TestAgent agent;
    
    // 测试设置和获取初始位置
    Eigen::Vector3d new_pos(4.0, 5.0, 6.0);
    agent.setInitialPos(new_pos);
    EXPECT_EQ(agent.getInitialPos(), new_pos);
    
    // 测试设置和获取当前位置
    Eigen::Vector3i current_pos(1, 2, 3);
    agent.setPos(current_pos);
    EXPECT_EQ(agent.getPos(), current_pos);
}

TEST(AgentTest, IDManagement)
{
    // 测试ID管理功能
    TestAgent agent1(100, Eigen::Vector3d::Zero());
    // 验证ID被正确设置
    EXPECT_GE(agent1.getID(), 0);
    
    // 测试重复ID处理（根据实现可能需要调整）
    TestAgent agent2(100, Eigen::Vector3d::Zero());
    // 根据Agent构造函数实现，可能会修改ID以避免重复
    EXPECT_NE(agent2.getID(), agent1.getID());
    
    // 创建第三个Agent，ID应该继续递增
    TestAgent agent3(100, Eigen::Vector3d::Zero());
    EXPECT_NE(agent3.getID(), agent1.getID());
    EXPECT_NE(agent3.getID(), agent2.getID());
}

// 测试UAV类
TEST(UAVTest, ConstructorAndStateManagement)
{
    // 测试构造函数
    UAV uav;
    EXPECT_EQ(uav.getState(), UAV::INIT);
    // UAV的ID应该是非负数
    EXPECT_GT(uav.getID(), 0);
    EXPECT_EQ(uav.getInitialPos(), Eigen::Vector3d::Zero());
    
    // 测试状态设置和获取
    uav.setState(UAV::READY);
    EXPECT_EQ(uav.getState(), UAV::READY);
    
    uav.setState(UAV::SEARCHING);
    EXPECT_EQ(uav.getState(), UAV::SEARCHING);
}

TEST(UAVTest, StateTransitions)
{
    // 测试UAV状态转换
    UAV uav;
    
    // 测试所有状态
    uav.setState(UAV::INIT);
    EXPECT_EQ(uav.getState(), UAV::INIT);
    
    uav.setState(UAV::READY);
    EXPECT_EQ(uav.getState(), UAV::READY);
    
    uav.setState(UAV::SEARCHING);
    EXPECT_EQ(uav.getState(), UAV::SEARCHING);
    
    uav.setState(UAV::MOVING);
    EXPECT_EQ(uav.getState(), UAV::MOVING);
    
    uav.setState(UAV::REACHED);
    EXPECT_EQ(uav.getState(), UAV::REACHED);
}

// 测试UGV类
TEST(UGVTest, ConstructorAndStateManagement)
{
    // 测试构造函数
    UGV ugv;
    EXPECT_EQ(ugv.getState(), UGV::INIT);
    // UGV的ID应该是非正数
    EXPECT_LT(ugv.getID(), 0);
    EXPECT_EQ(ugv.getInitialPos(), Eigen::Vector3d::Zero());
    
    // 测试状态设置和获取
    ugv.setState(UGV::READY);
    EXPECT_EQ(ugv.getState(), UGV::READY);
    
    ugv.setState(UGV::MOVING);
    EXPECT_EQ(ugv.getState(), UGV::MOVING);
}

TEST(UGVTest, PositionConstraint)
{
    // 测试UGV的Z坐标始终为0的约束
    UGV ugv;
    
    // 设置一个非零Z坐标的位姿
    Eigen::Vector3i pos(10, 20, 30);
    ugv.setPos(pos);
    
    // 检查Z坐标是否被强制设置为0
    Eigen::Vector3i actual_pos = ugv.getPos();
    EXPECT_EQ(actual_pos[0], 10);
    EXPECT_EQ(actual_pos[1], 20);
    EXPECT_EQ(actual_pos[2], 0);  // Z坐标应该始终为0
}

TEST(UGVTest, StateTransitions)
{
    // 测试UGV状态转换
    UGV ugv;
    
    // 测试所有状态
    ugv.setState(UGV::INIT);
    EXPECT_EQ(ugv.getState(), UGV::INIT);
    
    ugv.setState(UGV::READY);
    EXPECT_EQ(ugv.getState(), UGV::READY);
    
    ugv.setState(UGV::SEARCHING);
    EXPECT_EQ(ugv.getState(), UGV::SEARCHING);
    
    ugv.setState(UGV::MOVING);
    EXPECT_EQ(ugv.getState(), UGV::MOVING);
    
    ugv.setState(UGV::REACHED);
    EXPECT_EQ(ugv.getState(), UGV::REACHED);
}
