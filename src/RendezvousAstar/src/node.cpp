#include "node.h"

#include <ros/ros.h>
#include <utility>
namespace RendezvousAstar {

    std::atomic<int32_t> Node::version_{0};

    Node::Node(
        const int32_t pathID, const std::shared_ptr<Node>& parent, const int32_t x, const int32_t y, const int32_t z)
        : node_pos_(x, y, z) {
        addPath(pathID, INT32_MAX, 0, parent);
        state_[pathID] = UNUSED;
    }

    Node::Node(const int32_t pathID, const std::shared_ptr<Node>& parent, Eigen::Vector3i pos)
        : node_pos_(std::move(pos)) {
        addPath(pathID, INT32_MAX, 0, parent);
        state_[pathID] = UNUSED;
    }

    bool Node::addPath(const int32_t pathID, const double g, const double h, const std::shared_ptr<Node>& parent) {

        std::lock_guard<std::mutex> lock(mutex_);
        path_id_.insert(pathID);
        path_version_[pathID] = version_;
        g_[pathID]            = g;
        h_[pathID]            = h;
        parent_[pathID]       = parent;
        return true;
    }

    void Node::removePath(const int32_t path_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_id_.erase(path_id);
    }
    void Node::clearPath() {
        std::lock_guard<std::mutex> lock(mutex_);
        path_id_.clear();
    }

    bool Node::setG(const int32_t pathID, const double g) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node setG: 该路径不存在");
            return false;
        }
        g_[pathID] = g;
        return true;
    }

    bool Node::setH(const int32_t pathID, const double h) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node setH: 该路径不存在");
            return false;
        }
        h_[pathID] = h;
        return true;
    }

    bool Node::setParent(const int32_t pathID, const std::shared_ptr<Node>& parent) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node setParent: 该路径不存在");
            return false;
        }
        parent_[pathID] = parent;
        return true;
    }

    bool Node::setState(const int32_t pathID, const STATE state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_WARN("Node::setState:  路径id不在节点中");
            return false;
        }
        state_[pathID] = state;
        return true;
    }

    double Node::getG(const int32_t pathID) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end() || path_version_.at(pathID) != version_) {
            return INT32_MAX;
        }
        return g_.at(pathID);
    }

    Node::STATE Node::getState(int32_t pathID) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end() || path_version_.at(pathID) != version_) {
            return UNUSED;
        }
        return state_.at(pathID);
    }

    int32_t Node::getPathVersion(int32_t pathID) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end()) {
            return -1;
        }
        return path_version_.at(pathID);
    }

    double Node::getH(const int32_t pathID) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end() || path_version_.at(pathID) != version_) {
            ROS_ERROR("Node getH: 该路径不存在");
            return INT32_MAX;
        }
        return h_.at(pathID);
    }

    std::shared_ptr<Node> Node::getParent(const int32_t pathID) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_id_.find(pathID) == path_id_.end() || path_version_.at(pathID) != version_) {
            ROS_ERROR("Node getParent: 该路径不存在");
            return nullptr;
        }
        return parent_.at(pathID).lock();
    }

    Eigen::Vector3i Node::getPos() const {
        return node_pos_;
    }

    int32_t Node::getPathCount() const {
        return static_cast<int32_t>(path_id_.size());
    }

    bool Node::queryID(const int32_t id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return path_id_.find(id) != path_id_.end() && path_version_.at(id) == version_;
    }


} // namespace RendezvousAstar
