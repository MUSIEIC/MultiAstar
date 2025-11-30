#ifndef MULTIASTAR_NODE_MAP_H
#define MULTIASTAR_NODE_MAP_H

#include "node.h"
#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>

#include "voxel_map.hpp"
#include <boost/mpl/size.hpp>

namespace RendezvousAstar {

    /**
     * @brief 为Eigen::Vector3i提供哈希函数
     */
    struct NodeHash {
        /**
         * @brief 计算Eigen::Vector3i的哈希值
         * @param v 输入的三维整数向量
         * @return 哈希值
         */
        std::size_t operator()(const Eigen::Vector3i& v) const {
            std::size_t seed = 0;
            seed ^= std::hash<int>{}(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    /**
     * @brief 为Eigen::Vector3i提供相等比较函数
     */
    struct NodeEqual {
        /**
         * @brief 比较两个Eigen::Vector3i是否相等
         * @param lhs 左操作数
         * @param rhs 右操作数
         * @return 如果相等返回true，否则返回false
         */
        bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const {
            return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
        }
    };

    /**
     * @brief NodeMap类，用于管理节点地图
     */
    class NodeMap {
    private:
        /**
         * @brief 构造函数，私有化以实现单例模式
         */
        NodeMap() = default;

        static bool voxel_map_init; ///< 体素地图是否初始化标志
        static std::shared_ptr<voxel_map::VoxelMap> voxel_map_; ///< 体素地图指针
        std::unordered_map<Eigen::Vector3i, std::shared_ptr<Node>, NodeHash, NodeEqual> node_set_; ///< 节点集合
        static std::shared_mutex mutex_;

    public:
        /**
         * @brief 获取NodeMap单例实例
         * @return NodeMap实例的共享指针
         */
        static std::shared_ptr<NodeMap> getInstance() {
            static std::shared_ptr<NodeMap> instance(new NodeMap());

            return instance;
        }

        /**
         * @brief 析构函数
         */
        ~NodeMap() = default;

        /// 禁止拷贝构造函数
        NodeMap(const NodeMap&) = delete;

        /// 禁止移动构造函数
        NodeMap(NodeMap&&) = delete;

        /// 禁止拷贝赋值运算符
        NodeMap& operator=(const NodeMap&) = delete;

        /// 禁止移动赋值运算符
        NodeMap& operator=(NodeMap&&) = delete;

        /**
         * @brief 根据坐标获取节点
         * @param x X坐标
         * @param y Y坐标
         * @param z Z坐标
         * @return 节点的共享指针，如果不存在则返回nullptr
         */
        std::shared_ptr<Node> getNode(int32_t x, int32_t y, int32_t z) const;

        /**
         * @brief 根据坐标获取节点（强制）,不存在时创建
         * @param x X坐标
         * @param y Y坐标
         * @param z Z坐标
         * @param pathID 路径ID，仅用于创建节点
         * @param parent 父节点指针，默认值为nullptr,仅用于创建节点
         * @return 节点的共享指针的引用
         */
        std::shared_ptr<Node>& getNodeForce(int32_t x,int32_t y,int32_t z,int32_t pathID,const std::shared_ptr<Node>& parent=nullptr);

        /**
         * @brief 根据坐标向量获取节点
         * @param pos 坐标向量
         * @return 节点的共享指针，如果不存在则返回nullptr
         */
        std::shared_ptr<Node> getNode(const Eigen::Vector3i& pos) const;

        /**
         * @brief 根据坐标向量获取节点（强制）,不存在时创建
         * @param pos 坐标向量
         * @param pathID 路径ID，仅用于创建节点
         * @param parent 父节点指针，默认值为nullptr,仅用于创建节点
         * @return 节点的共享指针的引用
         */
        std::shared_ptr<Node>& getNodeForce(const Eigen::Vector3i& pos,int32_t pathID,const std::shared_ptr<Node>& parent=nullptr);


        /**
         * @brief 获取节点数量
         * @return 节点集合的大小
         */
        size_t getNodeNum() const;

        static bool voxelMapInitialized();

        static void initVoxelMap(const Eigen::Vector3i& size, const Eigen::Vector3d& origin, const double& voxScale);

        /**
         * @brief 打印节点集合信息（调试用）
         */
        void printNodeSet() const;

        void clear();
        /**
         * @brief 添加节点到节点集合中
         * @param node 要添加的节点
         */
        void addNode(const std::shared_ptr<Node>& node);

        /**
         * @brief 查询指定位置是否被占据
         * @param pos 三维浮点坐标位置
         * @return 如果被占据返回true，否则返回false
         */
        static bool query(const Eigen::Vector3d& pos);

        /**
         * @brief 查询指定位置是否被占据
         * @param pos 三维整数坐标位置
         * @return 如果被占据返回true，否则返回false
         */
        static bool query(const Eigen::Vector3i& pos);

        /**
         * @brief 将整数坐标转换为浮点坐标
         * @param id 整数坐标
         * @return 对应的浮点坐标
         */
        static Eigen::Vector3d posI2D(const Eigen::Vector3i& id);

        /**
         * @brief 将浮点坐标转换为整数坐标
         * @param pos 浮点坐标
         * @return 对应的整数坐标
         */
        static Eigen::Vector3i posD2I(const Eigen::Vector3d& pos);

        static void setOccupied(Eigen::Vector3d pos);

        static void dilate(const int &r);
    };

} // namespace RendezvousAstar

#endif // MULTIASTAR_NODE_MAP_H
