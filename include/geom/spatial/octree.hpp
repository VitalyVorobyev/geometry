#pragma once

#include <array>
#include <memory>
#include <vector>

#include "geom/primitives/point.hpp"
#include "geom/primitives/aabb.hpp"

namespace geom {
namespace spatial {

class Octree {
public:
    Octree(const AABB3d& bounds, std::size_t max_depth = 8, std::size_t max_points = 16)
        : bounds_(bounds), max_depth_(max_depth), max_points_(max_points) {}

    void build(const std::vector<Vec3>& pts) {
        points_ = &pts;
        root_ = std::make_unique<Node>(bounds_);
        root_->points.assign(points_->size(), 0);
        std::iota(root_->points.begin(), root_->points.end(), 0);
        subdivide(root_.get(), 0);
    }

    // Collect indices within an AABB region (broad-phase query)
    std::vector<std::size_t> query(const AABB3d& region) const {
        std::vector<std::size_t> out;
        query_recursive(root_.get(), region, out);
        return out;
    }

    // Collect representative points at a given depth for level-of-detail sampling
    std::vector<std::size_t> collect_level(std::size_t level) const {
        std::vector<std::size_t> out;
        collect_recursive(root_.get(), 0, level, out);
        return out;
    }

private:
    struct Node {
        explicit Node(const AABB3d& b) : box(b) {}
        AABB3d box;
        std::vector<std::size_t> points;
        std::array<std::unique_ptr<Node>, 8> child{};
        bool is_leaf() const { return child[0] == nullptr; }
    };

    const std::vector<Vec3>* points_{nullptr};
    AABB3d bounds_;
    std::size_t max_depth_; 
    std::size_t max_points_;
    std::unique_ptr<Node> root_;

    void subdivide(Node* node, std::size_t depth) {
        if (depth >= max_depth_ || node->points.size() <= max_points_) return;
        const Vec3 c = node->box.center();
        for (int i = 0; i < 8; ++i) {
            Vec3 min = node->box.min();
            Vec3 max = node->box.max();
            if (i & 1) min.x() = c.x(); else max.x() = c.x();
            if (i & 2) min.y() = c.y(); else max.y() = c.y();
            if (i & 4) min.z() = c.z(); else max.z() = c.z();
            node->child[i] = std::make_unique<Node>(AABB3d(min, max));
        }
        // distribute points
        for (auto idx : node->points) {
            const Vec3& p = (*points_)[idx];
            for (int i = 0; i < 8; ++i) {
                if (node->child[i]->box.contains(p)) {
                    node->child[i]->points.push_back(idx);
                    break;
                }
            }
        }
        node->points.clear();
        for (int i = 0; i < 8; ++i) subdivide(node->child[i].get(), depth + 1);
    }

    void query_recursive(const Node* node, const AABB3d& region, std::vector<std::size_t>& out) const {
        if (!node) return;
        if (!region.intersects(node->box)) return;
        if (node->is_leaf()) {
            out.insert(out.end(), node->points.begin(), node->points.end());
            return;
        }
        for (const auto& ch : node->child) query_recursive(ch.get(), region, out);
    }

    void collect_recursive(const Node* node, std::size_t depth, std::size_t target, std::vector<std::size_t>& out) const {
        if (!node) return;
        if (depth == target) {
            if (!node->points.empty()) out.push_back(node->points.front());
            return;
        }
        if (node->is_leaf()) return;
        for (const auto& ch : node->child) collect_recursive(ch.get(), depth + 1, target, out);
    }
};

} // namespace spatial
} // namespace geom

