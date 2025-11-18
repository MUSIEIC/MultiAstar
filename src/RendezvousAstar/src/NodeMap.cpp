#include <NodeMap.h>
#include <ros/ros.h>
namespace RendezvousAstar {

    /// 初始化体素地图
    std::mutex NodeMap::mutex_;
    std::shared_ptr<voxel_map::VoxelMap> NodeMap::voxel_map_;

    bool NodeMap::voxel_map_init = false;

    /**
     * @brief NodeMap构造函数
     */
    // NodeMap::NodeMap():
    // node_set_()
    // {
    //     // Eigen::Vector3i xyz(20,20,8);
    //     // Eigen::Vector3d offset(-2.5,-2.5,0.0);
    //     // voxel_map_=std::make_shared<voxel_map::VoxelMap>(xyz,offset,0.25);
    // }

    /**
     * @brief 根据坐标向量获取节点
     * @param pos 坐标向量
     * @return 节点的共享指针，如果不存在则返回nullptr
     */
    std::shared_ptr<Node> NodeMap::getNode(const Eigen::Vector3i& pos) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = node_set_.find(pos);
        if (it != node_set_.end()) {
            return it->second;
        } else {
            return nullptr;
        }
    }

    /**
     * @brief 根据坐标获取节点
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @return 节点的共享指针，如果不存在则返回nullptr
     */
    std::shared_ptr<Node> NodeMap::getNode(const int32_t x, const int32_t y, const int32_t z) const {
        return getNode(Eigen::Vector3i(x, y, z));
    }

    /**
     * @brief 获取体素地图
     * @return 体素地图的共享指针
     */
    std::shared_ptr<voxel_map::VoxelMap>& NodeMap::getVoxelMap() {
        return voxel_map_;
    }

    /**
     * @brief 获取节点数量
     * @return 节点集合的大小
     */
    size_t NodeMap::getNodeNum() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return node_set_.size();
    }

    bool NodeMap::voxelMapInitialized() {
        return voxel_map_init;
    }

    void NodeMap::initVoxelMap(const Eigen::Vector3i& size, const Eigen::Vector3d& origin, const double& voxScale) {
        if (!voxel_map_init) {
            voxel_map_     = std::make_shared<voxel_map::VoxelMap>(size, origin, voxScale);
            voxel_map_init = true;
        }
    }
    /**
     * @brief 打印节点集合信息（调试用）
     */
    void NodeMap::printNodeSet() const {
        for (const auto& it : node_set_) {
            ROS_WARN("节点位置：%d,%d,%d", it.first.x(), it.first.y(), it.first.z());
            // ROS_WARN("节点指针：%p",it.second.get());
        }
    }
    void NodeMap::clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        node_set_.clear();
    }
    /**
     * @brief 添加节点到节点集合中
     * @param node 要添加的节点
     */
    void NodeMap::addNode(const std::shared_ptr<Node>& node) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (node_set_.find(node->getPos()) != node_set_.end()) {
            ROS_WARN("该节点已在NodeMap中，不执行操作");
            return;
        }
        node_set_[node->getPos()] = node;
    }

    /**
     * @brief 查询指定位置是否被占据
     * @param pos 三维浮点坐标位置
     * @return 如果被占据返回true，否则返回false
     */
    bool NodeMap::query(const Eigen::Vector3d& pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        return voxel_map_->query(pos);
    }

    /**
     * @brief 查询指定位置是否被占据
     * @param pos 三维整数坐标位置
     * @return 如果被占据返回true，否则返回false
     */
    bool NodeMap::query(const Eigen::Vector3i& pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        return voxel_map_->query(pos);
    }

    /**
     * @brief 将整数坐标转换为浮点坐标
     * @param id 整数坐标
     * @return 对应的浮点坐标
     */
    Eigen::Vector3d NodeMap::posI2D(const Eigen::Vector3i& id) {
        std::lock_guard<std::mutex> lock(mutex_);
        return voxel_map_->posI2D(id);
    }

    /**
     * @brief 将浮点坐标转换为整数坐标
     * @param pos 浮点坐标
     * @return 对应的整数坐标
     */
    Eigen::Vector3i NodeMap::posD2I(const Eigen::Vector3d& pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        return voxel_map_->posD2I(pos);
    }


} // namespace RendezvousAstar
