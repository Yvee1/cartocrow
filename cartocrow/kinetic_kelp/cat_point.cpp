#include "cat_point.h"

namespace cartocrow::kinetic_kelp {
std::ostream& operator<<(std::ostream& os, CatPoint const& catPoint) {
	return os << catPoint.category << " " << catPoint.point;
}
}