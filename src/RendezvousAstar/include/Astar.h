#ifndef MULTIASTAR_ASTAR_H
#define MULTIASTAR_ASTAR_H
#include <Agent.h>
#include <Eigen/Eigen>
#include <functional>
#include <memory>
#include <vector>
namespace RendezvousAstar {
    class Astar {
    public:
        /**
         * @brief 构造函数
         * @param threshold 阈值，用于判断公共点数量是否足够
         * @param step 步数限制
         */
        explicit Astar(const int32_t threshold = 20, const int32_t step = 20) : threshold_(threshold), step_(step) {}

        /**
         * @brief 析构函数
         */
        ~Astar() = default;

        /**
         * @brief A*算法状态枚举
         */
        enum class STATE {
            searching, ///< 正在搜索
            reached, ///< 到达目标点
            common_over_threshold, ///< 公共点超过阈值
            reached_and_common, ///< 到达目标点且公共点满足条件
            map_search_done, ///< 地图搜索完成
            over_time ///< 超时
        };

        /**
         * @brief 执行一次A*算法迭代
         * @param agent 智能体指针
         * @param target 目标点坐标
         * @param nodeMap 节点地图指针
         * @param path_id 路径ID
         * @param path_num 路径数量
         * @return 当前搜索状态
         */
        STATE runOnce(std::shared_ptr<Agent>& agent, const Eigen::Vector3i& target,
            const std::shared_ptr<NodeMap>& nodeMap, int32_t path_id, int32_t path_num);

        /**
         * @brief 运行完整的A*算法
         * @param agent 智能体指针
         * @param target 目标点坐标
         * @param nodeMap 节点地图指针
         * @param path_id 路径ID
         * @param path_num 路径数量
         * @param end_condition 终止条件
         * @return 最终搜索状态
         */
        STATE run(std::shared_ptr<Agent>& agent, const Eigen::Vector3i& target, const std::shared_ptr<NodeMap>& nodeMap,
            int32_t path_id, int32_t path_num, const std::function<bool(STATE&)>& end_condition);

        /**
         * @brief 设置阈值
         * @param threshold 新的阈值
         */
        void setThreshold(int32_t threshold);

        /**
         * @brief 计算启发式函数值（H值）
         * @param start 起始点坐标
         * @param end_pos 终点坐标
         * @param method 计算方法，可选"euclidean"或"manhattan"
         * @return 启发式函数值
         */
        static double computeH(
            const Eigen::Vector3i& start, const Eigen::Vector3i& end_pos, const std::string& method = "euclidean");

        /**
         * @brief 获取栅格地图上的路径，非真实路径
         * @param path_id 路径ID
         * @param start 起点坐标
         * @param target 目标点坐标
         * @param nodeMap 节点地图指针
         * @return 路径(反向)
         */
        static std::vector<Eigen::Vector3i> getVoxelPath(int32_t path_id, const Eigen::Vector3i& start,
            const Eigen::Vector3i& target, const std::shared_ptr<NodeMap>& nodeMap);

        /**
         * @brief 获取真实路径
         * @param path_id 路径ID
         * @param start 起点坐标
         * @param target 终点坐标
         * @param nodeMap 节点地图指针
         * @return 真实路径(反向）
         */
        static std::vector<Eigen::Vector3d> getRealPath(int32_t path_id, const Eigen::Vector3i& start,
            const Eigen::Vector3i& target, const std::shared_ptr<NodeMap>& nodeMap);

        // 获取公共点数量
        size_t getCommonNum() const;

        /// 3D环境下的移动方向和代价
        std::vector<std::array<double, 4>> direct3d_ = {{{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}},
            {{0.0, 0.0, 1.0, 1.0}}, {{-1.0, 0.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}}, {{0.0, 0.0, -1.0, 1.0}},
            {{1.0, 1.0, 0.0, std::sqrt(2)}}, {{1.0, 0.0, 1.0, std::sqrt(2)}}, {{0.0, 1.0, 1.0, std::sqrt(2)}},
            {{-1.0, -1.0, 0.0, std::sqrt(2)}}, {{-1.0, 0.0, -1.0, std::sqrt(2)}}, {{0.0, -1.0, -1.0, std::sqrt(2)}},
            {{1.0, -1.0, 0.0, std::sqrt(2)}}, {{-1.0, 1.0, 0.0, std::sqrt(2)}}, {{1.0, 0.0, -1.0, std::sqrt(2)}},
            {{-1.0, 0.0, 1.0, std::sqrt(2)}}, {{0.0, 1.0, -1.0, std::sqrt(2)}}, {{0.0, -1.0, 1.0, std::sqrt(2)}},
            {{1.0, 1.0, 1.0, std::sqrt(3)}}, {{-1.0, -1.0, -1.0, std::sqrt(3)}}, {{1.0, 1.0, -1.0, std::sqrt(3)}},
            {{1.0, -1.0, 1.0, std::sqrt(3)}}, {{-1.0, 1.0, 1.0, std::sqrt(3)}}, {{-1.0, -1.0, 1.0, std::sqrt(3)}},
            {{-1.0, 1.0, -1.0, std::sqrt(3)}}, {{1.0, -1.0, -1.0, std::sqrt(3)}}}};

        /// 2D环境下的移动方向和代价
        std::vector<std::array<double, 4>> direct2d_ = {{{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}},
            {{-1.0, 0.0, 0.0, 1.0}}, {{0.0, -1.0, 0.0, 1.0}}, {{1.0, 1.0, 0.0, std::sqrt(2)}},
            {{1.0, -1.0, 0.0, std::sqrt(2)}}, {{-1.0, 1.0, 0.0, std::sqrt(2)}}, {{-1.0, -1.0, 0.0, std::sqrt(2)}}}};

    private:
        std::unordered_set<Eigen::Vector3i, NodeHash, NodeEqual> common_set_; ///< 存储公共点的集合
        int32_t threshold_; ///< 阈值
        int32_t step_; ///< 步数限制
    };

} // namespace RendezvousAstar

#endif // MULTIASTAR_ASTAR_H
