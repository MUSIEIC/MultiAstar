#ifndef NODE_H
#define NODE_H
#include <cfloat>
#include <Eigen/Eigen>
#include <memory>

namespace RendeAstar
{
    using ptrNode = std::shared_ptr<Node>;
    class Node
    {
    public:
        Node(int x = 0, int y = 0, int z = 0)
        {
            point_ = std::make_unique<Eigen::Vector3i>(x, y, z);
            g_.resize(pathNum_, DBL_MAX);
            totalG_ = 0;
            parent_.resize(pathNum_, std::weak_ptr<Node>());
            h_ = 0;
        };
        Node(std::unique_ptr<Eigen::Vector3i> point) : point_(std::move(point))
        {
            g_.resize(pathNum_, DBL_MAX);
            parent_.resize(pathNum_, std::weak_ptr<Node>());
            h_ = 0;
            totalG_ = 0;
        };

        ~Node(){};

        static void initPathNum(int pathNum)
        {
            pathNum_ = pathNum;
        }

        /*-----------------------------GetParament------------------------------------------*/
        const Eigen::Vector3i *getPoint() const
        {
            return point_.get();
        };

        const std::vector<double> getAllG() const
        {
            return g_;
        };

        const double getG(int pathid) const
        {
            return g_[pathid];
        }

        const double getH() const
        {
            return h_;
        };

        const Node *getParent(int pathid) const
        {
            return parent_[pathid].lock().get();
        };

        /*---------------------------------setParament-------------------------------------------------*/
        void setG(double g, int pathid)
        {
            g_[pathid] = g;
            totalG_ += g;
        }

        void setH(double h)
        {
            h_ = h;
        }

        void setParent(std::shared_ptr<Node> parent, int pathid)
        {
            parent_[pathid] = parent;
        }

        /*---------------------------------Function--------------------------------------------------*/

        ptrNode add(Eigen::Vector3i point,int pathid){
            std::unique_ptr<Eigen::Vector3i> nextpoint=std::make_unique<Eigen::Vector3i>(point+*point_);
            ptrNode nextnode = std::make_shared<Node>(nextpoint);
            nextnode->setG(g_[pathid] + sqrt(pow(point(0), 2) + pow(point(1), 2) + pow(point(2), 2)));
            return nextnode;
        }

        

        /*--------------------------------overload operator------------------------------------------*/

        bool operator==(const ptrNode &node)
        {
            return this->getPoint() == node->getPoint();
        }

    private:
        std::unique_ptr<Eigen::Vector3i> point_;
        std::vector<double> g_;
        double h_, totalG_;
        static int pathNum_;
        std::vector<std::weak_ptr<Node>> parent_;
    };
}

#endif