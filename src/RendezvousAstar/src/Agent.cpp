#include "Agent.h"

#include <fcntl.h>
#include <utility>

namespace RendezvousAstar {

    std::mutex Agent::id_set_mutex_;
    std::unordered_set<int32_t> Agent::id_set_;

    Agent::~Agent() {
        std::lock_guard<std::mutex> lock(id_set_mutex_);
        id_set_.erase(id_);
    }

    Agent::Agent(const int32_t id, Eigen::Vector3d initial_pos, const Eigen::Vector3i& pos)
        : id_(id), initial_pos_(std::move(initial_pos)) {
        {
            std::lock_guard<std::mutex> lock(id_set_mutex_);
            if (id_set_.find(id_) != id_set_.end()) {
                while (id_set_.find(id_) != id_set_.end()) {
                    ++id_;
                }
                ROS_WARN("该id以存在，已将id修改为：%d", id_);
            }
            id_set_.insert(id_);
        }
        pos_      = NodeMap::posD2I(initial_pos_);
        auto node = NodeMap::getInstance()->getNode(pos_);
        if (!node) {
            node = std::make_shared<Node>(id, nullptr, pos_);
            NodeMap::getInstance()->addNode(node);
        }
        node->addPath(id_, 0, 0, nullptr);
    }

    int32_t Agent::getID() const {
        return id_;
    }
    Eigen::Vector3d Agent::getInitialPos() const {
        return initial_pos_;
    }
    Eigen::Vector3i Agent::getPos() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return pos_;
    }


    void Agent::setPos(const Eigen::Vector3i& pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        pos_ = pos;
    }

    void Agent::setInitialPos(const Eigen::Vector3d& initial_pos) {
        initial_pos_ = initial_pos;
    }


    UAV::UAV() : state_(INIT) {
        std::lock_guard<std::mutex> lock(id_set_mutex_);
        if (id_ <= 0) {
            id_set_.erase(id_);
            if (id_ == 0) {
                id_ = 1;
            } else {
                id_ = -id_;
            }
            while (id_set_.find(id_) != id_set_.end()) {
                ++id_;
            }
            id_set_.insert(id_);
            ROS_WARN("UAV的id应为正数，已将id修改为：%d", id_);
        }
    }

    UAV::UAV(int32_t id, const Eigen::Vector3d& initial_pos, const Eigen::Vector3i& pos)
        : Agent(id, initial_pos, pos), state_(INIT) {
        std::lock_guard<std::mutex> lock(id_set_mutex_);
        if (id_ <= 0) {
            id_set_.erase(id_);
            if (id_ == 0) {
                id_ = 1;
            } else {
                id_ = -id_;
            }
            while (id_set_.find(id_) != id_set_.end()) {
                ++id_;
            }
            id_set_.insert(id_);
            ROS_WARN("UAV的id应为正数，已将id修改为：%d", id_);
        }
    }

    UAV::STATE UAV::getState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    Queue& UAV::getOpenList(const int32_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_list_;
    }


    void UAV::reset(int32_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = READY;
        open_list_.clear();

        auto node = NodeMap::getInstance()->getNode(pos_);
        if (!node) {
            node = std::make_shared<Node>(id, nullptr, pos_);
            NodeMap::getInstance()->addNode(node);
        }
        node->addPath(id, 0, 0, nullptr);
        open_list_.insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }


    void UAV::setState(const STATE& state) {
        state_ = state;
    }


    UGV::UGV() : state_(INIT) {
        std::lock_guard<std::mutex> lock(id_set_mutex_);
        setPos(pos_);
        if (id_ >= 0) {
            id_set_.erase(id_);
            if (id_ == 0) {
                id_ = -1;
            } else {
                id_ = -id_;
            }
            while (id_set_.find(id_) != id_set_.end()) {
                --id_;
            }
            id_set_.insert(id_);
            ROS_WARN("UGV的id应为负数，已将id修改为：%d", id_);
        }
    }

    UGV::UGV(int32_t id, const Eigen::Vector3d& initial_pos, const Eigen::Vector3i& pos)
        : Agent(id, initial_pos, pos), state_(INIT) {
        setPos(pos_);
        std::lock_guard<std::mutex> lock(id_set_mutex_);
        if (id_ >= 0) {
            id_set_.erase(id_);
            if (id_ == 0) {
                id_ = -1;
            } else {
                id_ = -id_;
            }
            while (id_set_.find(id_) != id_set_.end()) {
                --id_;
            }
            id_set_.insert(id_);
            ROS_WARN("UGV的id应为负数，已将id修改为：%d", id_);
        }
    }

    void UGV::setPos(const Eigen::Vector3i& pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        pos_ = Eigen::Vector3i(pos[0], pos[1], 0);
    }
    UGV::STATE UGV::getState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    Queue& UGV::getOpenList(const int32_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_list_[id];
    }


    void UGV::reset(int32_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = READY;
        open_list_[id].clear();

        auto node = NodeMap::getInstance()->getNode(pos_);
        if (!node) {
            node = std::make_shared<Node>(id, nullptr, pos_);
            NodeMap::getInstance()->addNode(node);
        }
        node->addPath(id, 0, 0, nullptr);
        ;
        open_list_[id].insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }


    void UGV::setState(const STATE& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
    }

} // namespace RendezvousAstar
