#include "misc.h"

namespace cartocrow::treemap {
bool approx_same_direction(Direction<Inexact> d1, Direction<Inexact> d2) {
	return abs(d1.dx()) < M_EPSILON && abs(d2.dx()) < M_EPSILON && d1.dy() * d2.dy() > 0 || abs(d1.dy() / d1.dx() - d2.dy() / d2.dx()) < M_EPSILON && d1.dx() * d2.dx() > 0;
}
}
