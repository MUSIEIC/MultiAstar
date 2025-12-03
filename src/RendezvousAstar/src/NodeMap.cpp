#include <NodeMap.h>
#include <ros/ros.h>
namespace RendezvousAstar {

    /// 初始化体素地图
    std::shared_mutex NodeMap::mutex_;
    std::shared_ptr<voxel_map::VoxelMap> NodeMap::voxel_map_;

    bool NodeMap::voxel_map_init = false;


    /**
     * @brief 根据坐标向量获取节点
     * @param pos 坐标向量
     * @return 节点的共享指针，如果不存在则返回nullptr
     */
    std::shared_ptr<Node> NodeMap::getNode(const Eigen::Vector3i& pos) const {
        std::shared_lock lock(mutex_);
        auto it = node_set_.find(pos);
        if (it != node_set_.end()) {
            return it->second;
        } else {
            return nullptr;
        }
    }
    std::shared_ptr<Node>& NodeMap::getNodeForce(
        const Eigen::Vector3i& pos, const int32_t pathID, const std::shared_ptr<Node>& parent) {
        {
            std::shared_lock lock(mutex_);
            auto it = node_set_.find(pos);
            if (it != node_set_.end()) {
                return it->second;
            }
        }
        {
            std::unique_lock lock(mutex_);
            auto it = node_set_.find(pos);
            if (it != node_set_.end()) {
                return it->second;
            }
            auto node      = std::make_shared<Node>(pathID, parent, pos);
            node_set_[pos] = node;
            return node_set_[pos];
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
    std::shared_ptr<Node>& NodeMap::getNodeForce(
        const int32_t x, const int32_t y, const int32_t z, const int32_t pathID, const std::shared_ptr<Node>& parent) {
        return getNodeForce(Eigen::Vector3i(x, y, z), pathID, parent);
    }


    /**
     * @brief 获取节点数量
     * @return 节点集合的大小
     */
    size_t NodeMap::getNodeNum() const {
        std::shared_lock lock(mutex_);
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
    void NodeMap::preInit() {
        const auto size = voxel_map_->getSize();
        std::unique_lock lock(mutex_);
        for (int i = 0; i < size.x(); i++) {
            for (int j = 0; j < size.y(); j++) {
                for (int k = 0; k < size.z(); k++) {
                    if (!voxel_map_->query(Eigen::Vector3i(i, j, k))) {
                        node_set_.emplace(Eigen::Vector3i(i, j, k), std::make_shared<Node>(0, nullptr, i, j, k));
                    }
                }
            }
        }
        ROS_INFO("地图节点预构建完成");
    }
    /**
     * @brief 打印节点集合信息（调试用）
     */
    void NodeMap::printNodeSet() const {
        for (const auto& it : node_set_) {
            ROS_WARN("节点位置：%d,%d,%d", it.first.x(), it.first.y(), it.first.z());
        }
    }
    void NodeMap::clear() {
        std::unique_lock lock(mutex_);
        node_set_.clear();
    }
    /**
     * @brief 添加节点到节点集合中
     * @param node 要添加的节点
     */
    void NodeMap::addNode(const std::shared_ptr<Node>& node) {
        std::unique_lock lock(mutex_);
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
        std::shared_lock lock(mutex_);
        return voxel_map_->query(pos);
    }

    /**
     * @brief 查询指定位置是否被占据
     * @param pos 三维整数坐标位置
     * @return 如果被占据返回true，否则返回false
     */
    bool NodeMap::query(const Eigen::Vector3i& pos) {
        std::shared_lock lock(mutex_);
        return voxel_map_->query(pos);
    }

    /**
     * @brief 将整数坐标转换为浮点坐标
     * @param id 整数坐标
     * @return 对应的浮点坐标
     */
    Eigen::Vector3d NodeMap::posI2D(const Eigen::Vector3i& id) {
        return voxel_map_->posI2D(id);
    }

    /**
     * @brief 将浮点坐标转换为整数坐标
     * @param pos 浮点坐标
     * @return 对应的整数坐标
     */
    Eigen::Vector3i NodeMap::posD2I(const Eigen::Vector3d& pos) {
        return voxel_map_->posD2I(pos);
    }

    void NodeMap::setOccupied(Eigen::Vector3d pos) {
        voxel_map_->setOccupied(pos);
    }

    void NodeMap::dilate(const int& r) {
        voxel_map_->dilate(r);
    }


} // namespace RendezvousAstar
