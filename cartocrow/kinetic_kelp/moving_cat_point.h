#ifndef CARTOCROW_MOVING_CAT_POINT_KK_H
#define CARTOCROW_MOVING_CAT_POINT_KK_H

#include "../core/core.h"

#include "trajectory.h"

namespace cartocrow::kinetic_kelp {
/// Categorical point
struct MovingCatPoint {
	unsigned int category;
	Trajectory trajectory;
	MovingCatPoint(unsigned int category, Trajectory trajectory) : category(category), trajectory(std::move(trajectory)) {};
	bool operator==(const MovingCatPoint&) const = default;
    MovingCatPoint transform(CGAL::Aff_transformation_2<Inexact>& t) const { return {category, trajectory.transform(t)}; }
};
}

#endif //CARTOCROW_MOVING_CAT_POINT_KK_H
