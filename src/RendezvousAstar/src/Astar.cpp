#include "Astar.h"

namespace RendezvousAstar {

    constexpr double sqrt2() {
        return std::sqrt(2);
    }
    constexpr double sqrt3() {
        return std::sqrt(3);
    }

    std::vector<std::array<double, 4>> Astar::direct3d26_ = {
        {{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}}, {{0.0, 0.0, 1.0, 1.0}}, {{-1.0, 0.0, 0.0, 1.0}},
            {{0.0, -1.0, 0.0, 1.0}}, {{0.0, 0.0, -1.0, 1.0}}, {{1.0, 1.0, 0.0, sqrt2()}}, {{1.0, 0.0, 1.0, sqrt2()}},
            {{0.0, 1.0, 1.0, sqrt2()}}, {{-1.0, -1.0, 0.0, sqrt2()}}, {{-1.0, 0.0, -1.0, sqrt2()}},
            {{0.0, -1.0, -1.0, sqrt2()}}, {{1.0, -1.0, 0.0, sqrt2()}}, {{-1.0, 1.0, 0.0, sqrt2()}},
            {{1.0, 0.0, -1.0, sqrt2()}}, {{-1.0, 0.0, 1.0, sqrt2()}}, {{0.0, 1.0, -1.0, sqrt2()}},
            {{0.0, -1.0, 1.0, sqrt2()}}, {{1.0, 1.0, 1.0, sqrt3()}}, {{-1.0, -1.0, -1.0, sqrt3()}},
            {{1.0, 1.0, -1.0, sqrt3()}}, {{1.0, -1.0, 1.0, sqrt3()}}, {{-1.0, 1.0, 1.0, sqrt3()}},
            {{-1.0, -1.0, 1.0, sqrt3()}}, {{-1.0, 1.0, -1.0, sqrt3()}}, {{1.0, -1.0, -1.0, sqrt3()}}}};

    std::vector<std::array<double, 4>> Astar::direct3d6_ = {{{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}},
        {{0.0, 0.0, 1.0, 1.0}}, {{-1.0, 0.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}}, {{0.0, 0.0, -1.0, 1.0}}}};


    std::vector<std::array<double, 4>> Astar::direct2d8_ = {{{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}},
        {{-1.0, 0.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}}, {{1.0, 1.0, 0.0, sqrt2()}}, {{1.0, -1.0, 0.0, sqrt2()}},
        {{-1.0, 1.0, 0.0, sqrt2()}}, {{-1.0, -1.0, 0.0, sqrt2()}}}};

    std::vector<std::array<double, 4>> Astar::direct2d4_ = {
        {{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}}, {{-1.0, 0.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}}};

    // 设置阈值，用于控制公共节点数量的上限
    void Astar::setThreshold(const int32_t threshold) {
        std::unique_lock lock(mutex_);
        threshold_ = threshold;
    }

    // 计算启发式函数值H，支持欧几里得距离和曼哈顿距离两种方法
    double Astar::computeH(const Eigen::Vector3i& start, const Eigen::Vector3i& end_pos, const std::string& method) {
        if (method == "euclidean") {
            // 欧几里得距离计算
            return (start.cast<double>() - end_pos.cast<double>()).norm();
        } else {
            // 曼哈顿距离计算
            return std::abs(start(0) - end_pos(0)) + std::abs(start(1) - end_pos(1)) + std::abs(start(2) - end_pos(2));
        }
    }

    // 根据路径ID获取体素路径（整数坐标）
    std::vector<Eigen::Vector3i> Astar::getVoxelPath(const int32_t path_id, const Eigen::Vector3i& start,
        const Eigen::Vector3i& target, const std::shared_ptr<NodeMap>& nodeMap) {
        auto node = nodeMap->getNode(target);
        if (node == nullptr) {
            ROS_WARN("该路径终点不在该路径id中");
            return {};
        }

        std::vector<Eigen::Vector3i> path;
        while (node) {
            path.emplace_back(node->getPos());
            if (node->getPos() == start) {
                break;
            }
            node = node->getParent(path_id);
        }
        return path;
    }


    // 根据路径ID获取实际路径（双精度浮点坐标）
    std::vector<Eigen::Vector3d> Astar::getRealPath(const int32_t path_id, const Eigen::Vector3i& start,
        const Eigen::Vector3i& target, const std::shared_ptr<NodeMap>& nodeMap) {
        auto node = nodeMap->getNode(target);
        if (node == nullptr) {
            ROS_WARN("该路径终点不在该路径id中");
            return {};
        }
        std::vector<Eigen::Vector3d> path;
        while (node) {
            path.emplace_back(NodeMap::posI2D(node->getPos()));
            if (node->getPos() == start) {
                break;
            }
            node = node->getParent(path_id);
        }
        return path;
    }


    bool Astar::isCommon(const std::shared_ptr<Node>& node, const std::vector<int32_t>& path_id_set) {
        if (path_id_set.empty()) {
            ROS_WARN("Astar::isCommon: path_id_set为空");
            return false;
        }
        bool ret = true;
        for (const auto& id : path_id_set) {
            ret &= node->queryID(id) && node->getState(id) == Node::STATE::INCLOSED;
        }
        if (ret) {
            for (const auto& id : path_id_set) {
                node->setState(id, Node::STATE::INCOMMONSET);
            }
        }
        return ret;
    }

    //  执行一次A*算法迭代
    Astar::STATE Astar::runOnce(std::shared_ptr<Agent>& agent, std::shared_ptr<Agent>& goal,
        const Eigen::Vector3i& desire_pos, const int32_t path_id, const std::vector<int32_t>& path_id_set,
        const bool use_more_directs) {

        auto nodeMap = NodeMap::getInstance();
        // 根据agent类型选择2D或3D方向数组
        std::vector<std::array<double, 4>>* direct;
        if (use_more_directs) {
            direct = &(typeid(*agent) == typeid(UAV) ? direct3d26_ : direct2d8_);
        } else {
            direct = &(typeid(*agent) == typeid(UAV) ? direct3d6_ : direct2d4_);
        }

        auto target = goal->getPos();
        // if (typeid(*agent) == typeid(UGV)) {
        //     target.z() = 0;
        // }

        auto state = STATE::searching;

        // 如果开放列表为空，说明搜索完成
        if (agent->openListEmpty()) {
            return STATE::map_search_done;
        }

        // 取出开放列表中f值最小的节点
        auto [f, x, y, z] = agent->openListPopTop();
        auto now          = nodeMap->getNode(Eigen::Vector3i(x, y, z));

        if (now->getState(path_id) == Node::STATE::INCLOSED || now->getState(path_id) == Node::STATE::INCOMMONSET) {
            return state;
        }

        if (computeH(now->getPos(), desire_pos) >= 10) {
            state = STATE::beyond_scope;
        }

        now->setState(path_id, Node::STATE::INCLOSED);

        // 如果当前节点被所有目标路径访问过，则加入公共集合
        if (isCommon(now, path_id_set)) {
            insertCommonSet(now);
        }
        // 判断是否到达目标点
        if (now->getPos() == target) {
            state = STATE::reached;
        }

        // 判断公共节点数量是否超过阈值
        if (getCommonNum() >= threshold_) {
            if (state == STATE::reached) {
                return STATE::reached_and_common;
            }
            state = STATE::common_over_threshold;
        }


        // 遍历当前节点的所有邻居节点
        for (auto& [dx, dy, dz, cost] : *direct) {
            int nx = x + dx, ny = y + dy, nz = z + dz;
            // 如果邻居节点是障碍物或已在关闭列表中，则跳过
            if (NodeMap::query(Eigen::Vector3i(nx, ny, nz))) {
                continue;
            }

            // 获取或创建邻居节点
            auto& node = nodeMap->getNodeForce(nx, ny, nz, path_id, now);
            // std::shared_ptr<Node> node = nodeMap->getNode(Eigen::Vector3i(nx, ny, nz));
            // if (!node) {
            //     node = std::make_shared<Node>(path_id, now, nx, ny, nz);
            //     nodeMap->addNode(node);
            // }
            if (node->getState(path_id) == Node::STATE::INCLOSED
                || node->getState(path_id) == Node::STATE::INCOMMONSET) {
                continue;
            }

            // 计算新的g值和h值
            double ng = now->getG(path_id) + cost, nh;
            if (now->getState(goal->getID()) == Node::STATE::INCLOSED) {
                nh = now->getG(goal->getID());
            } else {
                nh = 0.5 * computeH(node->getPos(), desire_pos) + 0.5 * computeH(node->getPos(), target);
            }


            // 如果新路径更优，则更新节点信息
            if (ng < node->getG(path_id)) {

                node->addPath(path_id, ng, nh, now, Node::STATE::INOPEN);
                agent->openListInsert(
                    {1.02 * nh + ng, static_cast<double>(nx), static_cast<double>(ny), static_cast<double>(nz)});
            }
        }

        return state;
    }

    // 运行完整的A*算法直到满足结束条件
    Astar::STATE Astar::run(std::shared_ptr<Agent>& agent, std::shared_ptr<Agent>& target,
        const Eigen::Vector3i& desire_pos, const int32_t path_id, const std::vector<int32_t>& path_id_set,
        const std::function<bool(STATE&)>& end_condition, const bool use_more_directs) {

        // 初始化
        auto state_now  = STATE::searching;
        auto begin_time = std::chrono::high_resolution_clock::now();

        // agent初始化
        agent->reset();

        // 循环执行A*算法直到满足结束条件或超时
        uint32_t cnt = 0;
        while (!end_condition(state_now)) {
            ++cnt;
            state_now     = runOnce(agent, target, desire_pos, path_id, path_id_set, use_more_directs);
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - begin_time);

            // 如果超过5秒则超时退出
            if (duration >= std::chrono::milliseconds(10000)) {
                state_now = STATE::over_time;
                ROS_WARN("Rendezvous::Astar::run: overtime >=10s");
                break;
            }
            if (state_now == STATE::map_search_done) {
                break;
            }
        }
        return state_now;
    }

} // namespace RendezvousAstar
