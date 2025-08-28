#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace geom {

#if defined(GEOM_SCALAR_DOUBLE)
using Scalar = double;
#else
using Scalar = float;
#endif

} // namespace geom
