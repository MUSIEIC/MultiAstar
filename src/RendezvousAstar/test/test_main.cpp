#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node_map");
    ros::NodeHandle nh;

    // 确保ROS节点正确初始化
    if (!ros::master::check()) {
        ROS_ERROR("No ROS master running, tests requiring ROS will fail");
        return 1;
    }
    
    auto result = RUN_ALL_TESTS();
    
    ros::shutdown();
    return result;
}