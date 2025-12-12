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

    Agent::Agent(const int32_t id, Eigen::Vector3d initial_pos, const double power)
        : id_(id), initial_pos_(std::move(initial_pos)), power_(power) {
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
        pos_ = NodeMap::posD2I(initial_pos_);
        // auto node = NodeMap::getInstance()->getNode(pos_);
        // if (!node) {
        //     node = std::make_shared<Node>(id, nullptr, pos_);
        //     NodeMap::getInstance()->addNode(node);
        // }
        const auto& node = NodeMap::getInstance()->getNodeForce(pos_, id);
        node->addPath(id_, 0, 0, nullptr, Node::STATE::INOPEN);
    }

    int32_t Agent::getID() const {
        return id_;
    }
    Eigen::Vector3d Agent::getInitialPos() const {
        return initial_pos_;
    }
    Eigen::Vector3i Agent::getPos() const {
        std::shared_lock lock(mutex_);
        return pos_;
    }
    double Agent::getPower() const {
        return power_;
    }


    void Agent::setPos(const Eigen::Vector3i& pos) {
        std::unique_lock lock(mutex_);
        pos_ = pos;
    }

    void Agent::setInitialPos(const Eigen::Vector3d& initial_pos) {
        std::unique_lock lock(mutex_);
        initial_pos_ = initial_pos;
    }
    void Agent::setPower(const double power) {
        power_ = power;
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

    UAV::UAV(int32_t id, const Eigen::Vector3d& initial_pos, const double power)
        : Agent(id, initial_pos, power), state_(INIT) {
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
        open_list_.insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }

    UAV::STATE UAV::getState() const {
        std::shared_lock lock(mutex_);
        return state_;
    }

    // Queue& UAV::getOpenList(const int32_t id) {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     return open_list_;
    // }


    void UAV::reset() {
        std::unique_lock lock(mutex_);
        state_ = READY;
        open_list_.clear();

        const auto& node = NodeMap::getInstance()->getNodeForce(pos_, id_);
        // auto node = NodeMap::getInstance()->getNode(pos_);
        // if (!node) {
        //     node = std::make_shared<Node>(id, nullptr, pos_);
        //     NodeMap::getInstance()->addNode(node);
        // }
        node->addPath(id_, 0, 0, nullptr, Node::STATE::INOPEN);
        open_list_.insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }


    void UAV::setState(const STATE& state) {
        state_ = state;
    }
    bool UAV::openListEmpty() {
        return open_list_.empty();
    }
    void UAV::openListErase(const std::array<double, 4>& element) {
        open_list_.erase(element);
    }
    std::array<double, 4> UAV::openListGetTop() {
        return *open_list_.begin();
    }
    std::array<double, 4> UAV::openListPopTop() {
        const auto begin = *open_list_.begin();
        open_list_.erase(begin);
        return begin;
    }
    void UAV::openListInsert(const std::array<double, 4>& element) {
        open_list_.insert(element);
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

    UGV::UGV(int32_t id, const Eigen::Vector3d& initial_pos, const double power)
        : Agent(id, initial_pos, power), state_(INIT) {
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
        open_list_.insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }

    void UGV::setPos(const Eigen::Vector3i& pos) {
        std::unique_lock lock(mutex_);
        pos_ = Eigen::Vector3i(pos[0], pos[1], 0);
    }
    UGV::STATE UGV::getState() const {
        std::shared_lock lock(mutex_);
        return state_;
    }

    // Queue& UGV::getOpenList(const int32_t id) {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     return open_list_[id];
    // }


    void UGV::reset() {
        std::unique_lock lock(mutex_);
        state_ = READY;
        open_list_.clear();

        const auto& node = NodeMap::getInstance()->getNodeForce(pos_, id_);
        // auto node = NodeMap::getInstance()->getNode(pos_);
        // if (!node) {
        //     node = std::make_shared<Node>(id, nullptr, pos_);
        //     NodeMap::getInstance()->addNode(node);
        // }
        node->addPath(id_, 0, 0, nullptr, Node::STATE::INOPEN);
        open_list_.insert(std::array<double, 4>{
            0.0, static_cast<double>(pos_[0]), static_cast<double>(pos_[1]), static_cast<double>(pos_[2])});
    }


    void UGV::setState(const STATE& state) {
        std::unique_lock lock(mutex_);
        state_ = state;
    }
    bool UGV::openListEmpty() {
        std::shared_lock lock(mutex_);
        return open_list_.empty();
    }
    void UGV::openListErase(const std::array<double, 4>& element) {
        std::unique_lock lock(mutex_);
        open_list_.erase(element);
    }
    void UGV::openListInsert(const std::array<double, 4>& element) {
        std::unique_lock lock(mutex_);
        open_list_.insert(element);
    }
    std::array<double, 4> UGV::openListGetTop() {
        std::unique_lock lock(mutex_);
        return *open_list_.begin();
    }
    std::array<double, 4> UGV::openListPopTop() {
        std::unique_lock lock(mutex_);
        const auto begin = *open_list_.begin();
        open_list_.erase(begin);
        return begin;
    }

} // namespace RendezvousAstar
