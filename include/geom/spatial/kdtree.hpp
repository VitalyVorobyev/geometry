#pragma once

#include <memory>
#include <vector>

#include "geom/primitives/point.hpp"
#include "geom/third_party/nanoflann.hpp"

namespace geom {
namespace spatial {

/** Simple 3D point cloud wrapper. */
struct PointCloud {
    std::vector<Vec3> points;
};

/** Adaptor exposing PointCloud to nanoflann. */
struct PointCloudAdaptor {
    const PointCloud* cloud{nullptr};
    explicit PointCloudAdaptor(const PointCloud* c = nullptr) : cloud(c) {}
    inline std::size_t kdtree_get_point_count() const { return cloud->points.size(); }
    inline Scalar kdtree_get_pt(std::size_t idx, int dim) const {
        return cloud->points[idx][static_cast<std::size_t>(dim)];
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

/** KD-tree wrapper powered by nanoflann. */
class KDTree {
public:
    using Index = std::size_t;
    using Adaptor = PointCloudAdaptor;
    using Dist = nanoflann::L2_Simple_Adaptor<Scalar, Adaptor>;
    using Impl = nanoflann::KDTreeSingleIndexAdaptor<Dist, Adaptor, 3, Index>;

    KDTree() = default;
    explicit KDTree(const PointCloud& cloud) { build(cloud); }

    void build(const PointCloud& cloud) {
        adaptor_ = Adaptor(&cloud);
        index_ = std::make_unique<Impl>(3, adaptor_);
        index_->buildIndex();
    }

    std::vector<Index> knn_search(const Vec3& query, std::size_t k) const {
        std::vector<Index> indices(k);
        std::vector<Scalar> dists(k);
        const std::size_t found = index_->knnSearch(query.data(), k, indices.data(), dists.data());
        indices.resize(found);
        return indices;
    }

    std::vector<Index> radius_search(const Vec3& query, Scalar radius) const {
        std::vector<std::pair<Index, Scalar>> matches;
        nanoflann::SearchParams params;
        index_->radiusSearch(query.data(), radius * radius, matches, params);
        std::vector<Index> result;
        result.reserve(matches.size());
        for (const auto& m : matches) result.push_back(m.first);
        return result;
    }

private:
    Adaptor adaptor_;
    std::unique_ptr<Impl> index_;
};

} // namespace spatial
} // namespace geom

