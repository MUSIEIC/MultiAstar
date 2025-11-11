#include "node.h"

#include <ros/ros.h>
#include <utility>
namespace RendezvousAstar {
    Node::Node(
        const int32_t pathID, const std::shared_ptr<Node>& parent, const int32_t x, const int32_t y, const int32_t z)
        : node_pos_(x, y, z) {
        addPath(pathID, INT32_MAX, 0, parent);
    }

    Node::Node(const int32_t pathID, const std::shared_ptr<Node>& parent, Eigen::Vector3i pos)
        : node_pos_(std::move(pos)) {
        addPath(pathID, INT32_MAX, 0, parent);
    }

    bool Node::addPath(const int32_t pathID, const double g, const double h, const std::shared_ptr<Node>& parent) {
        if (path_id_.find(pathID) != path_id_.end()) {
            ROS_WARN("Node: 当前加入的pathID在该节点已存在");
            return false;
        }
        path_id_.insert(pathID);
        g_[pathID]      = g;
        h_[pathID]      = h;
        parent_[pathID] = parent;
        return true;
    }

    bool Node::setG(const int32_t pathID, const double g) {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
            return false;
        }
        g_[pathID] = g;
        return true;
    }

    bool Node::setH(const int32_t pathID, const double h) {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
            return false;
        }
        h_[pathID] = h;
        return true;
    }

    bool Node::setParent(const int32_t pathID, const std::shared_ptr<Node>& parent) {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
            return false;
        }
        parent_[pathID] = parent;
        return true;
    }

    double Node::getG(const int32_t pathID) const {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
            return INT32_MAX;
        }
        return g_.at(pathID);
    }

    double Node::getH(const int32_t pathID) const {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
            return INT32_MAX;
        }
        return h_.at(pathID);
    }

    std::shared_ptr<Node> Node::getParent(const int32_t pathID) const {
        if (path_id_.find(pathID) == path_id_.end()) {
            ROS_ERROR("Node: 该路径不存在");
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
        return path_id_.find(id) != path_id_.end();
    }


} // namespace RendezvousAstar
