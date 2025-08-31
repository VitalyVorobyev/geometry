#pragma once

#include <algorithm>
#include <array>
#include <stack>
#include <vector>

#include "geom/algorithms/intersect.hpp"
#include "geom/primitives/aabb.hpp"
#include "geom/primitives/triangle.hpp"
#include "geom/primitives/ray.hpp"

namespace geom {
namespace spatial {

class BVH {
public:
    explicit BVH(const std::vector<Triangle3d>& tris) { build(tris); }
    BVH() = default;

    void build(const std::vector<Triangle3d>& tris) {
        tris_ = &tris;
        indices_.resize(tris.size());
        std::iota(indices_.begin(), indices_.end(), 0);
        nodes_.clear();
        nodes_.reserve(tris.size() * 2);
        root_ = build_node(0, tris.size());
    }

    // Ray traversal using iterative stack
    bool intersect(const Ray3d& r, Scalar& t, std::size_t& idx) const {
        if (root_ < 0) return false;
        bool hit = false;
        t = std::numeric_limits<Scalar>::infinity();
        std::stack<int> stack;
        stack.push(root_);
        while (!stack.empty()) {
            int nidx = stack.top();
            stack.pop();
            const Node& node = nodes_[nidx];
            if (!ray_box_intersect(r, node.box, Scalar(0), t)) continue;
            if (node.leaf()) {
                for (std::size_t i = node.start; i < node.end; ++i) {
                    std::size_t triIdx = indices_[i];
                    auto hitRes = intersect(r, (*tris_)[triIdx]);
                    if (hitRes.hit && hitRes.t < t) {
                        t = hitRes.t;
                        idx = triIdx;
                        hit = true;
                    }
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        return hit;
    }

private:
    struct Node {
        AABB3d box;
        int left{-1}, right{-1};
        std::size_t start{0}, end{0};
        bool leaf() const { return left < 0 && right < 0; }
    };

    const std::vector<Triangle3d>* tris_{nullptr};
    std::vector<std::size_t> indices_;
    std::vector<Node> nodes_;
    int root_{-1};

    int build_node(std::size_t begin, std::size_t end) {
        Node node;
        node.start = begin;
        node.end = end;
        AABB3d box((*tris_)[indices_[begin]].a);
        for (std::size_t i = begin; i < end; ++i) {
            const auto& tri = (*tris_)[indices_[i]];
            box.extend(tri.a);
            box.extend(tri.b);
            box.extend(tri.c);
        }
        node.box = box;
        int idx = static_cast<int>(nodes_.size());
        nodes_.push_back(node);
        if (end - begin <= 2) return idx; // leaf
        // choose split axis by largest extent
        Vec3 ext = box.sizes();
        int axis = 0;
        if (ext.y() > ext.x()) axis = 1;
        if (ext.z() > ext[axis]) axis = 2;
        std::size_t mid = (begin + end) / 2;
        auto comp = [&](std::size_t a, std::size_t b) {
            Vec3 ca = ((*tris_)[a].a + (*tris_)[a].b + (*tris_)[a].c) / Scalar(3);
            Vec3 cb = ((*tris_)[b].a + (*tris_)[b].b + (*tris_)[b].c) / Scalar(3);
            return ca[axis] < cb[axis];
        };
        std::nth_element(indices_.begin() + begin, indices_.begin() + mid, indices_.begin() + end, comp);
        nodes_[idx].left = build_node(begin, mid);
        nodes_[idx].right = build_node(mid, end);
        return idx;
    }

    static bool ray_box_intersect(const Ray3d& r, const AABB3d& box, Scalar tmin, Scalar tmax) {
        for (int i = 0; i < 3; ++i) {
            Scalar invD = Scalar(1) / r.dir[i];
            Scalar t0 = (box.min()[i] - r.origin[i]) * invD;
            Scalar t1 = (box.max()[i] - r.origin[i]) * invD;
            if (invD < Scalar(0)) std::swap(t0, t1);
            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;
            if (tmax <= tmin) return false;
        }
        return true;
    }
};

} // namespace spatial
} // namespace geom

