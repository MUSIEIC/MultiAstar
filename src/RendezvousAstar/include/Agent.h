#ifndef MULTIASTAR_AGENT_H
#define MULTIASTAR_AGENT_H

#include <NodeMap.h>
#include <array>
#include <ros/ros.h>
#include <set>
#include <unordered_set>
namespace RendezvousAstar {
    using Queue = std::set<std::array<double, 4>>;
    using List  = std::unordered_set<Eigen::Vector3i, NodeHash, NodeEqual>;

    /**
     * @brief Agent类，表示一个智能体(agent)，是UAV和UGV的基类
     */
    class Agent {
    public:
        /**
         * @brief 构造函数
         * @param id 智能体ID，默认为0
         * @param initial_pos 初始位置，默认为零向量
         * @param power 能量，百分比，默认为1.0
         */
        explicit Agent(int32_t id = 0, Eigen::Vector3d initial_pos = Eigen::Vector3d::Zero(), double power = 1.0f);

        /**
         * @brief 虚析构函数
         */
        virtual ~Agent();

        /**
         * @brief 获取智能体ID
         * @return 智能体ID
         */
        int32_t getID() const;

        /**
         * @brief 获取初始位置
         * @return 初始位置
         */
        Eigen::Vector3d getInitialPos() const;

        /**
         * @brief 获取当前位置
         * @return 当前位置
         */
        Eigen::Vector3i getPos() const;

        double getPower() const;
        // virtual Queue& getOpenList(int32_t path_id) = 0;


        virtual void reset() = 0;

        /**
         * @brief 设置当前位置
         * @param pos 新的位置
         */
        virtual void setPos(const Eigen::Vector3i& pos);

        /**
         * @brief 设置初始位置
         * @param initial_pos 新的初始位置
         */
        virtual void setRealPos(const Eigen::Vector3d& initial_pos);

        void setPower(const double power);

        virtual bool openListEmpty() = 0;

        virtual std::array<double, 4> openListGetTop() = 0;

        virtual std::array<double, 4> openListPopTop() = 0;

        virtual void openListErase(const std::array<double, 4>& element) = 0;

        virtual void openListInsert(const std::array<double, 4>& element) = 0;


    protected:
        static std::unordered_set<int32_t> id_set_; ///< 存储已使用的ID集合，防止重复ID
        int32_t id_; ///< 智能体唯一标识符
        Eigen::Vector3d real_pos_; ///< 智能体初始位置（三维浮点坐标）
        Eigen::Vector3i pos_; ///< 智能体当前位置（三维整数坐标）
        mutable std::shared_mutex mutex_;
        static std::mutex id_set_mutex_;
        double power_;
    };


    /**
     * @brief UAV类，表示无人机，继承自Agent类
     */
    class UAV final : public Agent {
    public:
        /**
         * @brief UAV状态枚举
         */
        enum STATE {
            INIT, ///< 初始化状态
            READY, ///< 准备就绪状态
            SEARCHING, ///< 搜索状态
            MOVING, ///< 移动状态
            REACHED ///< 到达目标状态
        };

        /**
         * @brief 默认构造函数
         */
        explicit UAV();

        /**
         * @brief 带参数的构造函数
         * @param id 智能体ID
         * @param initial_pos 初始位置
         * @param power 电量
         */
        UAV(int32_t id, const Eigen::Vector3d& initial_pos = Eigen::Vector3d::Zero(), double power = 1.0);

        /**
         * @brief 虚析构函数
         */
        ~UAV() override = default;

        /**
         * @brief 获取当前状态
         * @return 当前状态
         */
        STATE getState() const;

        // Queue& getOpenList(int32_t id) override;
        void reset() override;
        /**
         * @brief 设置当前状态
         * @param state 新的状态
         */
        void setState(const STATE& state);

        bool openListEmpty() override;

        void openListErase(const std::array<double, 4>& element) override;

        std::array<double, 4> openListGetTop() override;

        std::array<double, 4> openListPopTop() override;

        void openListInsert(const std::array<double, 4>& element) override;

    private:
        STATE state_; ///< UAV当前状态
        Queue open_list_;
    };

    /**
     * @brief UGV类，表示地面车辆，继承自Agent类
     */
    class UGV final : public Agent {
    public:
        /**
         * @brief UGV状态枚举
         */
        enum STATE {
            INIT, ///< 初始化状态
            READY, ///< 准备就绪状态
            SEARCHING, ///< 搜索状态
            MOVING, ///< 移动状态
            REACHED ///< 到达目标状态
        };

        /**
         * @brief 默认构造函数
         */
        explicit UGV();

        /**
         * @brief 带参数的构造函数
         * @param id 智能体ID
         * @param initial_pos 初始位置
         * @param power 表示电量
         */
        UGV(int32_t id, const Eigen::Vector3d& initial_pos = Eigen::Vector3d::Zero(),double high = 0.2,double power = 1.0);

        /**
         * @brief 虚析构函数
         */
        ~UGV() override = default;

        /**
         * @brief 获取当前状态
         * @return 当前状态
         */
        STATE getState() const;

        // Queue& getOpenList(int32_t id) override;
        void reset() override;

        /**
         * @brief 设置当前位置（重写父类方法）
         * @param pos 新的位置
         */
        void setPos(const Eigen::Vector3i& pos) override;

        void setRealPos(const Eigen::Vector3d& pos) override;

        /**
         * @brief 设置当前状态
         * @param state 新的状态
         */
        void setState(const STATE& state);

        void setRealHigh(double  high);

        void setHigh(int high);

        double getRealHigh() const;

        int getHigh() const;

        bool openListEmpty() override;

        void openListErase(const std::array<double, 4>& element) override;

        void openListInsert(const std::array<double, 4>& element) override;

        std::array<double, 4> openListGetTop() override;

        std::array<double, 4> openListPopTop() override;

    private:
        STATE state_; ///< UGV当前状态
        Queue open_list_;
        double high_real_;
        int  high_;
    };

} // namespace RendezvousAstar

#endif // MULTIASTAR_AGENT_H
