#ifndef MULTIASTAR_NODE_H
#define MULTIASTAR_NODE_H

#include <Eigen/Eigen>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
namespace RendezvousAstar {

    /**
     * @brief A*算法中的节点类，用于存储路径搜索过程中的节点信息
     *
     * 该类维护了节点的位置信息以及多个路径的相关数据，
     * 包括g值(起始点到当前节点的实际代价)、h值(当前节点到目标点的启发式估计代价)
     * 以及父节点指针等信息。
     */
    class Node {
    public:
        static std::atomic<int32_t> version_;

        enum STATE { UNUSED, INOPEN, INCLOSED, INCOMMONSET };
        /**
         * @brief 构造函数，使用坐标和路径ID初始化节点
         * @param pathID 路径标识符
         * @param parent 父节点的智能指针
         * @param x x坐标，默认为0
         * @param y y坐标，默认为0
         * @param z z坐标，默认为0
         */
        explicit Node(int32_t pathID, const std::shared_ptr<Node>& parent, int32_t x = 0, int32_t y = 0, int32_t z = 0);

        /**
         * @brief 构造函数，使用位置元组和路径ID初始化节点
         * @param pathID 路径标识符
         * @param parent 父节点的智能指针
         * @param pos 包含(x,y,z)坐标的Eigen::Vector3i
         */
        Node(int32_t pathID, const std::shared_ptr<Node>& parent, Eigen::Vector3i pos);

        ~Node() = default;

        /**
         * @brief 为指定路径添加节点信息
         * @param pathID 路径标识符
         * @param g 从起点到当前节点的实际代价
         * @param h 从当前节点到终点的启发式估计代价
         * @param parent 当前节点在该路径上的父节点
         * @return 添加成功返回true，否则返回false
         */
        bool addPath(int32_t pathID, double g, double h, const std::shared_ptr<Node>& parent);

        void removePath(int32_t path_id);

        void clearPath();

        void setPath(int32_t pathID, double g, double h, const std::shared_ptr<Node>& parent);
        /**
         * @brief 设置指定路径的h值
         * @param pathID 路径标识符
         * @param h 启发式估计代价
         * @return 设置成功返回true，否则返回false
         */
        bool setH(int32_t pathID, double h);

        /**
         * @brief 设置指定路径的g值
         * @param pathID 路径标识符
         * @param g 实际代价
         * @return 设置成功返回true，否则返回false
         */
        bool setG(int32_t pathID, double g);

        /**
         * @brief 设置指定路径的父节点
         * @param pathID 路径标识符
         * @param parent 父节点的智能指针
         * @return 设置成功返回true，否则返回false
         */
        bool setParent(int32_t pathID, const std::shared_ptr<Node>& parent);

        bool setState(int32_t pathID, STATE state);
        /**
         * @brief 获取指定路径的h值
         * @param pathID 路径标识符
         * @return 对应路径的h值
         */
        double getH(int32_t pathID) const;

        /**
         * @brief 获取指定路径的g值
         * @param pathID 路径标识符
         * @return 对应路径的g值
         */
        double getG(int32_t pathID) const;

        STATE getState(int32_t pathID) const;

        int32_t getPathVersion(int32_t pathID) const;
        /**
         * @brief 获取指定路径的父节点
         * @param pathID 路径标识符
         * @return 对应路径父节点的智能指针
         */
        std::shared_ptr<Node> getParent(int32_t pathID) const;

        Eigen::Vector3i getPos() const;

        int32_t getPathCount() const;

        bool queryID(int32_t id) const;

        std::unordered_set<int32_t> getPathSet() const;

    private:
        // 节点的三维坐标位置
        const Eigen::Vector3i node_pos_;
        // 存储经过此节点的所有路径ID集合
        std::unordered_set<int32_t> path_id_;

        // 存储各路径对应的g值映射表
        std::unordered_map<int32_t, double> g_;
        // 存储各路径对应的h值映射表
        std::unordered_map<int32_t, double> h_;

        std::unordered_map<int32_t, STATE> state_;

        std::unordered_map<int32_t, int32_t> path_version_;
        // pathID对应的父节点
        std::unordered_map<int32_t, std::weak_ptr<Node>> parent_;
        mutable std::shared_mutex mutex_;
    };

} // namespace RendezvousAstar

#endif // MULTIASTAR_NODE_H
