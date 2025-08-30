#pragma once

#include <algorithm>
#include <array>

#include "geom/primitives/point.hpp"

namespace geom {

struct OBB3d {
    Vec3 center{Vec3::Zero()};
    std::array<Vec3,3> axes{Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};
    Vec3 half_extents{Vec3::Ones()*Scalar(0.5)};

    bool contains(const Vec3& p) const {
        Vec3 d = p - center;
        for(int i=0;i<3;++i) {
            Scalar dist = d.dot(axes[i]);
            if (std::abs(dist) > half_extents[i]) return false;
        }
        return true;
    }

    Vec3 project_point(const Vec3& p) const {
        Vec3 d = p - center;
        Vec3 result = center;
        for(int i=0;i<3;++i) {
            Scalar dist = d.dot(axes[i]);
            Scalar clamped = std::clamp(dist, -half_extents[i], half_extents[i]);
            result += clamped * axes[i];
        }
        return result;
    }

    Scalar signed_distance(const Vec3& p) const {
        Vec3 d = p - center;
        Vec3 q;
        for(int i=0;i<3;++i) {
            Scalar dist = d.dot(axes[i]);
            q[i] = std::abs(dist) - half_extents[i];
        }
        Vec3 q_pos = q.cwiseMax(Vec3::Zero());
        Scalar outside = q_pos.norm();
        Scalar inside = std::min(q.maxCoeff(), Scalar(0));
        return outside + inside;
    }
};

} // namespace geom

