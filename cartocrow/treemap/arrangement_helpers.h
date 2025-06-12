#ifndef CARTOCROW_ARRANGEMENT_HELPERS_H
#define CARTOCROW_ARRANGEMENT_HELPERS_H

#include "treemap_helpers.h"
#include "misc.h"

namespace cartocrow::treemap {
typedef CGAL::Arr_walk_along_line_point_location<TMArrangement> TM_pl;

Polygon<K> faces_to_polygon(const std::unordered_set<FaceH>& faces);
/// Create an arrangement of a single rectangle. Returns the bounded face.
std::pair<std::shared_ptr<TMArrangement>, FaceH> arrangement_rectangle(const Rectangle<K>& rect);
FaceConstH getFaceOf(const TMArrangement& arr, Point<K> centroid);
bool isVertical(const HalfedgeH& e);
bool isHorizontal(const HalfedgeH& e);
std::optional<HalfedgeH> nextOnMaximalSegment(const HalfedgeH& e);
std::optional<HalfedgeH> prevOnMaximalSegment(const HalfedgeH& e);

struct MaximalSegment {
	Segment<K> segment;
	std::vector<HalfedgeH> halfedges;
};
}

#endif //CARTOCROW_ARRANGEMENT_HELPERS_H
