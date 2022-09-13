// 参考： https://rosettacode.org/wiki/K-d_tree
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

template<typename coordsType, size_t dims>
class PointNd
{
private:
    std::array<coordsType, dims> coords_;

public:
    PointNd(std::array<coordsType, dims> co) : coords_(co) {}
    PointNd(std::initializer_list<coordsType> list) {
        size_t n = std::min(dims, list.size());
        std::copy_n(list.begin(), n, coords_.begin());
    }

    coordsType getCoordinates(size_t idx) const {
        return coords_[idx];
    }

    double calcDistance(const PointNd &pt) const {
        double dist = 0;
        for (size_t i = 0; i < dims; ++i) {
            double d = getCoordinates(i) - pt.getCoordinates(i);
            dist += d * d;
        }
        return dist;
    }
}; // class PointNd

template<typename coordsType, size_t dims>
struct KdNode
{
    PointNd<coordsType, dims> point;
    KdNode *left;
    KdNode *right;

    KdNode(const PointNd<coordsType, dims> &pt) : point(pt), left(nullptr), right(nullptr) {}

    coordsType getCoordinates(size_t idx) const {
        return point.getCoordinates(idx);
    }

    double calcDistance(const PointNd<coordsType, dims> &pt) const {
        return point.calcDistance(pt);
    }
}; // struct KdNode

template<typename coordsType, size_t dims>
struct NodeComparer
{
    size_t idx;
    explicit NodeComparer(size_t idx_) : idx(idx_) {}
    bool operator()(const KdNode<coordsType, dims> &nl, const KdNode<coordsType, dims> &nr) const {
        return nl.point.getCoordinates(idx) < nr.point.getCoordinates(idx);
    }
}; // struct NodeComparer

template<typename coordsType, size_t dims>
class KdTree
{
private:
    KdNode<coordsType, dims> *root_ = nullptr;
    KdNode<coordsType, dims> *best_ = nullptr;
    double best_dist_ = 0;
    size_t visited_ = 0;
    std::vector<KdNode<coordsType, dims>> nodes_;

    KdNode<coordsType, dims>* makeTree(size_t begin, size_t end, size_t idx) {
        if (end <= begin)
            return nullptr;
        size_t n = begin + (end - begin) / 2;
        auto i = nodes_.begin();
        std::nth_element(i + begin, i + n, i + end, NodeComparer<coordsType, dims>(idx));
        idx = (idx + 1) % dims;
        nodes_[n].left = makeTree(begin, n, idx);
        nodes_[n].right = makeTree(n + 1, end, idx);
        return &nodes_[n];
    }

    void nnSearchRecursive(KdNode<coordsType, dims> *root, const PointNd<coordsType, dims> &point, size_t idx) {
        if (root == nullptr)
            return;
        ++visited_;
        double d = root->calcDistance(point);
        if (best_ == nullptr || d < best_dist_) {
            best_dist_ = d;
            best_ = root;
        }
        if (best_dist_ == 0)
            return;
        double dx = root->getCoordinates(idx) - point.getCoordinates(idx);
        idx = (idx + 1) % dims;
        nnSearchRecursive(dx > 0 ? root->left : root->right, point, idx);
        if (dx * dx >= best_dist_)
            return;
        nnSearchRecursive(dx > 0 ? root->right : root->left, point, idx);
    }

public:
    KdTree(const KdTree<coordsType, dims>&) = delete;
    KdTree<coordsType, dims>& operator=(const KdTree<coordsType, dims>&) = delete;
    template<typename iterator>
    KdTree(iterator begin, iterator end) : nodes_(begin, end) {
        root_ = makeTree(0, nodes_.size(), 0);
    }
    template<typename func>
    KdTree(func&& f, size_t n) {
        nodes_.reserve(n);
        for (size_t i = 0; i < n; ++i)
            nodes_.push_back(f());
        root_ = makeTree(0, nodes_.size(), 0);
    }

    bool empty() const {return nodes_.empty();}
    size_t visited() const {return visited_;}
    double distance() const {return std::sqrt(best_dist_);}
    const PointNd<coordsType, dims>& nnSearch(const PointNd<coordsType, dims> &pt) {
        if (root_ == nullptr)
            throw std::logic_error("tree is empty");
        best_ = nullptr;
        visited_ = 0;
        best_dist_ = 0;
        nnSearchRecursive(root_, pt, 0);
        return best_->point;
    }

}; // class KdTree