#include <random>
#include <iostream>

#include "kdtree.hpp"

typedef PointNd<double, 3> Point3d;
typedef KdTree<double, 3> Tree3d;

template<typename coordsType, size_t dims>
std::ostream& operator<<(std::ostream &out, const PointNd<coordsType, dims> &pt) {
    out << '(';
    for (size_t i = 0; i < dims; ++i) {
        if (i > 0)
            out << ", ";
        out << pt.getCoordinates(i);
    }
    out << ')';
    return out;
}

class RandomPointGenerator
{
private:
    std::mt19937 engine_;
    std::uniform_real_distribution<double> distrib_;

public:
    RandomPointGenerator(double minval, double maxval) : engine_(std::random_device()()), distrib_(minval, maxval) {}

    Point3d operator()() {
        double x = distrib_(engine_);
        double y = distrib_(engine_);
        double z = distrib_(engine_);
        return Point3d({x, y, z});
    }
};

int main() {
    size_t count = 1000;
    RandomPointGenerator rpg(0, 1);
    Tree3d tree(rpg, count);
    Point3d pt(rpg());
    Point3d n = tree.nnSearch(pt);

    std::cout << "Random data (" << count << " pts):\n";
    std::cout << "point: " << pt << '\n';
    std::cout << "nearest point: " << n << '\n';
    std::cout << "distance: " << tree.distance() << '\n';
    std::cout << "nodes visited: " << tree.visited() << '\n';

    return 0;
}