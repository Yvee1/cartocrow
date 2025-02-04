#ifndef CARTOCROW_CAT_POINT_KK_H
#define CARTOCROW_CAT_POINT_KK_H

#include "../core/core.h"

namespace cartocrow::kinetic_kelp {
/// Categorical point
struct CatPoint {
	unsigned int category;
	Point<Exact> point;
	CatPoint(unsigned int category, Point<Exact> point) : category(category), point(std::move(point)) {};
	bool operator==(const CatPoint&) const = default;
};

std::ostream& operator<<(std::ostream& os, CatPoint const& catPoint);
}

#endif //CARTOCROW_CAT_POINT_KK_H
