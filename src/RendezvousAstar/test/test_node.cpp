#include <gtest/gtest.h>
#include "node.h"
#include <limits>

using namespace RendezvousAstar;

TEST(NodeTest, ConstructorAndBasicAccess)
{
    // 测试构造函数和基本访问功能
    auto parentNode = std::make_shared<Node>(0, nullptr, 1, 2, 3);
    Node node(1, parentNode, 4, 5, 6);

    EXPECT_EQ(node.getPos(), Eigen::Vector3i(4, 5, 6));
}

TEST(NodeTest, ConstructorWithVector)
{
    // 测试使用Vector3i的构造函数
    auto parentNode = std::make_shared<Node>(0, nullptr, Eigen::Vector3i(1, 2, 3));
    Eigen::Vector3i pos(4, 5, 6);
    Node node(1, parentNode, pos);

    EXPECT_EQ(node.getPos(), pos);
}

TEST(NodeTest, PathManagement)
{
    // 测试路径管理功能
    auto parentNode = std::make_shared<Node>(0, nullptr, 0, 0, 0);
    Node node(1, parentNode, 1, 1, 1);

    // 测试添加路径
    EXPECT_TRUE(node.addPath(2, 10.0, 5.0, parentNode));

    // 测试重复添加路径应该失败
    EXPECT_FALSE(node.addPath(2, 15.0, 3.0, parentNode));

    // 测试获取路径数据
    EXPECT_DOUBLE_EQ(node.getG(2), 10.0);
    EXPECT_DOUBLE_EQ(node.getH(2), 5.0);
    EXPECT_EQ(node.getParent(2), parentNode);
}

TEST(NodeTest, PathDataModification)
{
    // 测试路径数据修改功能
    auto parentNode = std::make_shared<Node>(0, nullptr, 0, 0, 0);
    auto newParentNode = std::make_shared<Node>(0, nullptr, 1, 1, 1);
    Node node(1, parentNode, 1, 1, 1);

    node.addPath(2, 10.0, 5.0, parentNode);

    // 测试设置G值
    EXPECT_TRUE(node.setG(2, 15.0));
    EXPECT_DOUBLE_EQ(node.getG(2), 15.0);

    // 测试设置H值
    EXPECT_TRUE(node.setH(2, 8.0));
    EXPECT_DOUBLE_EQ(node.getH(2), 8.0);

    // 测试设置父节点
    EXPECT_TRUE(node.setParent(2, newParentNode));
    EXPECT_EQ(node.getParent(2), newParentNode);

    // 测试对不存在的路径操作应该失败
    EXPECT_FALSE(node.setG(3, 10.0));
    EXPECT_FALSE(node.setH(3, 10.0));
    EXPECT_FALSE(node.setParent(3, parentNode));

    // 测试获取不存在路径的数据应该返回默认值
    EXPECT_DOUBLE_EQ(node.getG(3), INT32_MAX);
    EXPECT_DOUBLE_EQ(node.getH(3), INT32_MAX);
    EXPECT_EQ(node.getParent(3), nullptr);
}

TEST(NodeTest, MultiplePaths)
{
    // 测试多个路径的管理
    auto parentNode1 = std::make_shared<Node>(0, nullptr, 0, 0, 0);
    auto parentNode2 = std::make_shared<Node>(1, nullptr, 1, 1, 1);
    Node node(0, nullptr, 2, 2, 2);

    // 添加两个不同的路径
    EXPECT_TRUE(node.addPath(1, 10.0, 5.0, parentNode1));
    EXPECT_TRUE(node.addPath(2, 15.0, 7.0, parentNode2));

    // 验证两个路径的数据都正确
    EXPECT_DOUBLE_EQ(node.getG(1), 10.0);
    EXPECT_DOUBLE_EQ(node.getH(1), 5.0);
    EXPECT_EQ(node.getParent(1), parentNode1);

    EXPECT_DOUBLE_EQ(node.getG(2), 15.0);
    EXPECT_DOUBLE_EQ(node.getH(2), 7.0);
    EXPECT_EQ(node.getParent(2), parentNode2);
}

TEST(NodeTest, EdgeCases)
{
    // 测试边界情况
    Node node(1, nullptr, 0, 0, 0);  // 使用非0的pathID避免冲突

    // 测试负数值
    EXPECT_TRUE(node.addPath(2, -5.0, -3.0, nullptr));
    EXPECT_DOUBLE_EQ(node.getG(2), -5.0);
    EXPECT_DOUBLE_EQ(node.getH(2), -3.0);

    // 测试零值
    EXPECT_TRUE(node.setG(2, 0.0));
    EXPECT_TRUE(node.setH(2, 0.0));
    EXPECT_DOUBLE_EQ(node.getG(2), 0.0);
    EXPECT_DOUBLE_EQ(node.getH(2), 0.0);
}

// 新增测试用例：大量路径测试
TEST(NodeTest, LargeNumberOfPaths)
{
    Node node(1000, nullptr, 0, 0, 0);  // 使用一个不太可能冲突的pathID
    
    // 添加大量路径
    const int numPaths = 100;
    std::vector<std::shared_ptr<Node>> parents;
    
    for (int i = 1; i <= numPaths; ++i) {
        auto parent = std::make_shared<Node>(i+2000, nullptr, i, i, i);  // 避免ID冲突
        parents.push_back(parent);
        
        EXPECT_TRUE(node.addPath(i+1000, i * 10.0, i * 5.0, parent)) << "Failed to add path with ID: " << (i+1000);
        EXPECT_DOUBLE_EQ(node.getG(i+1000), i * 10.0) << "G value mismatch for ID: " << (i+1000);
        EXPECT_DOUBLE_EQ(node.getH(i+1000), i * 5.0) << "H value mismatch for ID: " << (i+1000);
        EXPECT_EQ(node.getParent(i+1000), parent) << "Parent mismatch for ID: " << (i+1000);
    }
    
    // 确保所有路径都正确添加
    EXPECT_EQ(parents.size(), numPaths);
}

// 新增测试用例：特殊值测试
TEST(NodeTest, SpecialValues)
{
    Node node(1000, nullptr, 0, 0, 0);  // 使用非0的pathID避免冲突
    
    // 测试极大值
    EXPECT_TRUE(node.addPath(2000, DBL_MAX, DBL_MAX, nullptr));
    EXPECT_DOUBLE_EQ(node.getG(2000), DBL_MAX);
    EXPECT_DOUBLE_EQ(node.getH(2000), DBL_MAX);
    
    // 测试极小值
    EXPECT_TRUE(node.setG(2000, -DBL_MAX));
    EXPECT_TRUE(node.setH(2000, -DBL_MAX));
    EXPECT_DOUBLE_EQ(node.getG(2000), -DBL_MAX);
    EXPECT_DOUBLE_EQ(node.getH(2000), -DBL_MAX);
    
    // 测试NaN值
    EXPECT_TRUE(node.setG(2000, NAN));
    EXPECT_TRUE(node.setH(2000, NAN));
    EXPECT_TRUE(std::isnan(node.getG(2000)));
    EXPECT_TRUE(std::isnan(node.getH(2000)));
    
    // 测试无穷大值
    EXPECT_TRUE(node.setG(2000, INFINITY));
    EXPECT_TRUE(node.setH(2000, INFINITY));
    EXPECT_TRUE(std::isinf(node.getG(2000)));
    EXPECT_TRUE(std::isinf(node.getH(2000)));
}

// 新增测试用例：路径ID边界测试
TEST(NodeTest, PathIDBoundaryConditions)
{
    Node node(1000, nullptr, 0, 0, 0);  // 使用非0的pathID避免冲突
    
    // 测试边界ID值 (避免使用0和1，因为构造函数和测试中可能已经用了)
    std::vector<int32_t> testIDs = {INT32_MIN, -100, 100, INT32_MAX};
    
    for (size_t i = 0; i < testIDs.size(); ++i) {
        int32_t id = testIDs[i];
        auto parent = std::make_shared<Node>(id+2000, nullptr, i+10, i+10, i+10);  // 避免ID冲突
        EXPECT_TRUE(node.addPath(id, (i+10) * 10.0, (i+10) * 5.0, parent)) << "Failed to add path with ID: " << id;
        EXPECT_DOUBLE_EQ(node.getG(id), (i+10) * 10.0) << "G value mismatch for ID: " << id;
        EXPECT_DOUBLE_EQ(node.getH(id), (i+10) * 5.0) << "H value mismatch for ID: " << id;
        EXPECT_EQ(node.getParent(id), parent) << "Parent mismatch for ID: " << id;
    }
}

// 新增测试用例：空指针父节点测试
TEST(NodeTest, NullParentNode)
{
    Node node(1000, nullptr, 0, 0, 0);  // 使用非0的pathID避免冲突
    
    // 测试添加空指针父节点
    EXPECT_TRUE(node.addPath(2000, 10.0, 5.0, nullptr));
    EXPECT_DOUBLE_EQ(node.getG(2000), 10.0);
    EXPECT_DOUBLE_EQ(node.getH(2000), 5.0);
    EXPECT_EQ(node.getParent(2000), nullptr);
    
    // 测试修改为空指针父节点
    auto parentNode = std::make_shared<Node>(3000, nullptr, 1, 1, 1);
    EXPECT_TRUE(node.addPath(3000, 20.0, 10.0, parentNode));
    EXPECT_EQ(node.getParent(3000), parentNode);
    
    EXPECT_TRUE(node.setParent(3000, nullptr));
    EXPECT_EQ(node.getParent(3000), nullptr);
}