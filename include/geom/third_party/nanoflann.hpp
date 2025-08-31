#pragma once

#include <algorithm>
#include <array>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

#include "geom/primitives/point.hpp"
#include "geom/primitives/aabb.hpp"

namespace nanoflann {

/** Minimal L2 adaptor providing element type definition. */
template <typename T, class DatasetAdaptor>
struct L2_Simple_Adaptor {
    using ElementType = T;
};

struct SearchParams {
    int checks{32};
    float eps{0.0f};
    bool sorted{true};
};

struct KDTreeSingleIndexAdaptorParams {
    explicit KDTreeSingleIndexAdaptorParams(int leaf_max_size = 10)
        : leaf_max_size(leaf_max_size) {}
    int leaf_max_size;
};

/**
 * Simplified KD-tree implementation providing the same entry points as
 * nanoflann::KDTreeSingleIndexAdaptor for basic kNN and radius queries.
 * This is a lightweight stand-in used when the real nanoflann library is
 * unavailable.
 */
template <typename Distance, class DatasetAdaptor, int DIM = 3,
          typename IndexType = std::size_t>
class KDTreeSingleIndexAdaptor {
public:
    using Index = IndexType;
    using Scalar = typename Distance::ElementType;

    KDTreeSingleIndexAdaptor(int /*dim*/, const DatasetAdaptor& adaptor,
                             const KDTreeSingleIndexAdaptorParams& params =
                                 KDTreeSingleIndexAdaptorParams())
        : dataset_(adaptor), leaf_size_(params.leaf_max_size) {
        buildIndex();
    }

    void buildIndex() {
        const size_t n = dataset_.kdtree_get_point_count();
        indices_.resize(n);
        std::iota(indices_.begin(), indices_.end(), Index(0));
        nodes_.clear();
        nodes_.reserve(n);
        root_ = build_recursive(0, n, 0);
    }

    size_t knnSearch(const Scalar* query_pt, const size_t num_closest,
                     Index* out_indices, Scalar* out_dist_sq) const {
        std::priority_queue<std::pair<Scalar, Index>> heap;
        const geom::Vec3 q(query_pt[0], query_pt[1], query_pt[2]);
        knn_recursive(root_, q, num_closest, heap);
        const size_t found = heap.size();
        for (size_t i = 0; i < found; ++i) {
            out_indices[found - 1 - i] = heap.top().second;
            out_dist_sq[found - 1 - i] = heap.top().first;
            heap.pop();
        }
        return found;
    }

    size_t radiusSearch(const Scalar* query_pt, const Scalar radius_sq,
                        std::vector<std::pair<Index, Scalar>>& ret_matches,
                        const SearchParams& = SearchParams()) const {
        const geom::Vec3 q(query_pt[0], query_pt[1], query_pt[2]);
        radius_recursive(root_, q, radius_sq, ret_matches);
        if (!ret_matches.empty() && ret_matches.size() > 1) {
            std::sort(ret_matches.begin(), ret_matches.end(),
                      [](const auto& a, const auto& b) {
                          return a.second < b.second;
                      });
        }
        return ret_matches.size();
    }

private:
    struct Node {
        Index index{0};
        int axis{0};
        Scalar split{0};
        int left{-1};
        int right{-1};
        geom::AABB3d bbox;
        bool leaf{false};
    };

    const DatasetAdaptor& dataset_;
    std::vector<Index> indices_;
    std::vector<Node> nodes_;
    int root_{-1};
    int leaf_size_{10};

    int build_recursive(Index begin, Index end, int depth) {
        if (begin >= end) return -1;
        const Index count = end - begin;
        const int axis = depth % 3;
        const Index mid = begin + count / 2;
        auto comp = [&](Index a, Index b) {
            return dataset_.kdtree_get_pt(a, axis) <
                   dataset_.kdtree_get_pt(b, axis);
        };
        std::nth_element(indices_.begin() + begin, indices_.begin() + mid,
                         indices_.begin() + end, comp);
        const int node_index = static_cast<int>(nodes_.size());
        nodes_.push_back({indices_[mid], axis,
                          dataset_.kdtree_get_pt(indices_[mid], axis), -1, -1,
                          {}, count <= leaf_size_});
        Node& node = nodes_.back();
        geom::Vec3 vmin(std::numeric_limits<Scalar>::infinity(),
                        std::numeric_limits<Scalar>::infinity(),
                        std::numeric_limits<Scalar>::infinity());
        geom::Vec3 vmax(-std::numeric_limits<Scalar>::infinity(),
                        -std::numeric_limits<Scalar>::infinity(),
                        -std::numeric_limits<Scalar>::infinity());
        for (Index i = begin; i < end; ++i) {
            geom::Vec3 p(dataset_.kdtree_get_pt(indices_[i], 0),
                         dataset_.kdtree_get_pt(indices_[i], 1),
                         dataset_.kdtree_get_pt(indices_[i], 2));
            vmin = vmin.cwiseMin(p);
            vmax = vmax.cwiseMax(p);
        }
        node.bbox = geom::AABB3d(vmin, vmax);
        if (count > leaf_size_) {
            node.left = build_recursive(begin, mid, depth + 1);
            node.right = build_recursive(mid + 1, end, depth + 1);
        }
        return node_index;
    }

    void knn_recursive(int node_idx, const geom::Vec3& q, size_t k,
                       std::priority_queue<std::pair<Scalar, Index>>& heap) const {
        if (node_idx < 0) return;
        const Node& node = nodes_[node_idx];
        const geom::Vec3 pt(dataset_.kdtree_get_pt(node.index, 0),
                             dataset_.kdtree_get_pt(node.index, 1),
                             dataset_.kdtree_get_pt(node.index, 2));
        const Scalar dist2 = (pt - q).squaredNorm();
        if (heap.size() < k) {
            heap.emplace(dist2, node.index);
        } else if (dist2 < heap.top().first) {
            heap.pop();
            heap.emplace(dist2, node.index);
        }
        const Scalar diff = q[node.axis] - node.split;
        const int first = diff < 0 ? node.left : node.right;
        const int second = diff < 0 ? node.right : node.left;
        if (first >= 0) knn_recursive(first, q, k, heap);
        const Scalar radius2 = heap.size() < k
                                   ? std::numeric_limits<Scalar>::infinity()
                                   : heap.top().first;
        if (second >= 0 && diff * diff < radius2)
            knn_recursive(second, q, k, heap);
    }

    void radius_recursive(int node_idx, const geom::Vec3& q, Scalar r2,
                          std::vector<std::pair<Index, Scalar>>& out) const {
        if (node_idx < 0) return;
        const Node& node = nodes_[node_idx];
        if (!node.bbox.contains(q) &&
            node.bbox.squaredExteriorDistance(q) > r2) return;
        const geom::Vec3 pt(dataset_.kdtree_get_pt(node.index, 0),
                             dataset_.kdtree_get_pt(node.index, 1),
                             dataset_.kdtree_get_pt(node.index, 2));
        const Scalar dist2 = (pt - q).squaredNorm();
        if (dist2 <= r2) out.emplace_back(node.index, dist2);
        if (node.left >= 0) radius_recursive(node.left, q, r2, out);
        if (node.right >= 0) radius_recursive(node.right, q, r2, out);
    }
};

} // namespace nanoflann

