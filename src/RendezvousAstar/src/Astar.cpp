#include "Astar.h"

namespace RendezvousAstar {

    // 设置阈值，用于控制公共节点数量的上限
    void Astar::setThreshold(const int32_t threshold) {
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

    // 获取公共节点集合的大小
    size_t Astar::getCommonNum() const {
        return common_set_.size();
    }

    //  执行一次A*算法迭代
    Astar::STATE Astar::runOnce(std::shared_ptr<Agent>& agent, const Eigen::Vector3i& target,
        const std::shared_ptr<NodeMap>& nodeMap, const int32_t path_id, const int32_t path_num) {
        // 根据agent类型选择2D或3D方向数组
        auto& direct       = typeid(*agent) == typeid(UAV) ? direct3d_ : direct2d_;
        auto& open_list    = agent->getOpenList(path_id);
        auto& closed_list  = agent->getClosedList(path_id);
        auto& in_open_list = agent->getInOpenList(path_id);
        auto reach_state   = STATE::searching;

        // 如果开放列表为空，说明搜索完成
        if (open_list.empty()) {
            return STATE::map_search_done;
        }

        // 取出开放列表中f值最小的节点
        auto [f, x, y, z] = *open_list.begin();
        auto now          = nodeMap->getNode(Eigen::Vector3i(x, y, z));
        auto begin        = open_list.begin();
        open_list.erase(begin);
        in_open_list.erase(Eigen::Vector3i(begin->at(1), begin->at(2), begin->at(3)));
        closed_list.insert(Eigen::Vector3i(x, y, z));

        // 判断是否到达目标点
        if (now->getPos() == target) {
            reach_state = STATE::reached;
        }

        // 判断公共节点数量是否超过阈值
        if (getCommonNum() >= threshold_) {
            if (reach_state == STATE::reached) {
                return STATE::reached_and_common;
            } else {
                reach_state = STATE::common_over_threshold;
            }
        }

        // 如果当前节点被足够多的路径访问过且在地面层，则加入公共集合
        if (now->getPathCount() >= path_num && now->getPos()[2] == 0) {
            common_set_.insert(now->getPos());
        }

        // 遍历当前节点的所有邻居节点
        for (auto& [dx, dy, dz, cost] : direct) {
            int nx = x + dx, ny = y + dy, nz = z + dz;
            // 如果邻居节点是障碍物或已在关闭列表中，则跳过
            if (nodeMap->query(Eigen::Vector3i(nx, ny, nz)) || closed_list.count(Eigen::Vector3i(nx, ny, nz))) {
                // if (closed_list.count(Eigen::Vector3i(nx,ny,nz)))
                continue;
            }

            // 获取或创建邻居节点
            std::shared_ptr<Node> node = nodeMap->getNode(Eigen::Vector3i(nx, ny, nz));
            if (!node) {
                node = std::make_shared<Node>(path_id, now, nx, ny, nz);
                nodeMap->addNode(node);
            }

            // 计算新的g值和h值
            double ng = now->getG(path_id) + cost;
            double nh = computeH(node->getPos(), target);

            if (in_open_list.find(node->getPos()) == in_open_list.end()) {
                node->setG(path_id, INT_MAX);
            }

            // 如果新路径更优，则更新节点信息
            if (nh + ng < node->getG(path_id) + node->getH(path_id)) {
                open_list.erase({node->getG(path_id) + node->getH(path_id), static_cast<double>(nx),
                    static_cast<double>(ny), static_cast<double>(nz)});
                node->setG(path_id, ng);
                node->setH(path_id, nh);
                node->setParent(path_id, now);
                open_list.insert({nh + ng, static_cast<double>(nx), static_cast<double>(ny), static_cast<double>(nz)});
                in_open_list.insert(node->getPos());
            }
        }

        return reach_state;
    }

    // 运行完整的A*算法直到满足结束条件
    Astar::STATE Astar::run(std::shared_ptr<Agent>& agent, const Eigen::Vector3i& target,
        const std::shared_ptr<NodeMap>& nodeMap, const int32_t path_id, const int32_t path_num,
        const std::function<bool(STATE&)>& end_condition) {

        // 初始化
        auto state_now  = STATE::searching;
        auto begin_time = std::chrono::high_resolution_clock::now();

        // agent初始化
        agent->getOpenList(path_id).clear();
        agent->getInOpenList(path_id).clear();
        agent->getOpenList(path_id).insert(std::array<double, 4>{0.0, static_cast<double>(agent->getPos()[0]),
            static_cast<double>(agent->getPos()[1]), static_cast<double>(agent->getPos()[2])});
        agent->getInOpenList(path_id).insert(agent->getPos());
        agent->getClosedList(path_id).clear();


        // 循环执行A*算法直到满足结束条件或超时
        uint32_t cnt = 0;
        while (!end_condition(state_now)) {
            ++cnt;
            state_now     = runOnce(agent, target, nodeMap, path_id, path_num);
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - begin_time);

            // 如果超过5秒则超时退出
            if (duration >= std::chrono::milliseconds(5000)) {
                state_now = STATE::over_time;
                ROS_WARN("Rendezvous::Astar::run: overtime >=5s");
                break;
            }
            if (state_now == STATE::map_search_done) {
                break;
            }
        }
        ROS_INFO("Rendezvous::Astar::run: cnt=%d,closed_list size: %lu", cnt, agent->getClosedList(path_id).size());
        return state_now;
    }

} // namespace RendezvousAstar
