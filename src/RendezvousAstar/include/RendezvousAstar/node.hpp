#ifndef NODE_H
#define NODE_H
#include <cfloat>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <unordered_map>
#include <assert.h>
namespace RendeAstar
{
    using ptrNode = std::shared_ptr<Node>;

    class Node
    {
    public:
        Node(int x = 0, int y = 0, int z = 0)
        {
            point_ = Eigen::Vector3i(x, y, z);
            g_.resize(pathNum_, DBL_MAX);
            totalG_ = 0;
            parent_.resize(pathNum_, std::weak_ptr<Node>());
            h_ = 0;
        };
        Node(Eigen::Vector3i point) : point_(std::move(point))
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
        const Eigen::Vector3i &getPoint() const
        {
            return point_;
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

        const ptrNode getParent(int pathid) const
        {
            return parent_[pathid].lock();
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

        ptrNode add(std::pair<Eigen::Vector3i, double> point, int pathid)
        {
            Eigen::Vector3i nextpoint = point.first + point_;
            ptrNode nextnode = std::make_shared<Node>(nextpoint);
            nextnode->setG(g_[pathid] + point.second);
            return nextnode;
        }

        /*--------------------------------overload operator------------------------------------------*/

        bool operator==(const ptrNode &node)
        {
            return this->getPoint() == node->getPoint();
        }

    private:
        Eigen::Vector3i point_;
        std::vector<double> g_;
        double h_, totalG_;
        static int pathNum_;
        std::vector<std::weak_ptr<Node>> parent_;
    };
    class NodeManager
    {
    public:
        NodeManager(const NodeManager &) = delete;
        NodeManager &operator=(const NodeManager &) = delete;
        NodeManager(NodeManager &&) = delete;
        NodeManager &operator=(NodeManager &&) = delete;

        NodeManager()
        {
            for (int i = -1; i <= 1; ++i)
            {
                for (int j = -1; j <= 1; ++j)
                {
                    for (int k = -1; k <= 1; ++k)
                    {
                        if (i == 0 && j == 0 && k == 0)
                            continue;
                        motions3D_.emplace_back(Eigen::Vector3i(i, j, k), std::sqrt(i * i + j * j + k * k));
                    }
                }
            }
            for (int i = -1; i <= 1; ++i)
            {
                for (int j = -1; j <= 1; ++j)
                {
                    if (i == 0 && j == 0)
                        continue;
                    motions2D_.emplace_back(Eigen::Vector3i(i, j, 0), std::sqrt(i * i + j * j));
                }
            }
        }

        std::vector<std::pair<Eigen::Vector3i, double>> &motions3D()
        {
            return motions3D_;
        };

        std::vector<std::pair<Eigen::Vector3i, double>> &motions2D()
        {
            return motions2D_;
        }

        void addNode(const ptrNode &node)
        {
            if (nodeMap_.count(node->getPoint()))
            {
                throw std::runtime_error("Node already exists in the map.");
            }
            nodeMap_[node->getPoint()] = node;
        }

        bool haveNode(const Eigen::Vector3i &pos, ptrNode &node)
        {
            auto it = nodeMap_.find(pos);
            if (it != nodeMap_.end())
            {
                node = it->second;
                return true;
            }
            else
                return false;
        }

        /*----------------------------------customize-----------------------------------*/

        struct hashNode
        {
            std::size_t operator()(const Eigen::Vector3i &vec) const
            {

                std::hash<int> hasher;
                size_t seed = 0;
                seed ^= hasher(vec.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= hasher(vec.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= hasher(vec.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                return seed;
            }
        };

    private:
        std::vector<std::pair<Eigen::Vector3i, double>> motions3D_;
        std::vector<std::pair<Eigen::Vector3i, double>> motions2D_;
        std::unordered_map<Eigen::Vector3i, ptrNode, hashNode> nodeMap_;
    };
}

#endif