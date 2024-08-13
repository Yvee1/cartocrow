#ifndef CARTOCROW_ARRANGEMENT_HELPERS_H
#define CARTOCROW_ARRANGEMENT_HELPERS_H

#include "treemap_helpers.h"
namespace cartocrow::treemap {
typedef CGAL::Arr_walk_along_line_point_location<TMArrangement> TM_pl;

Polygon<K> faces_to_polygon(const std::unordered_set<FaceH>& faces);
std::pair<std::shared_ptr<TMArrangement>, FaceH> arrangement_rectangle(const Rectangle<K>& rect);
FaceConstH getFaceOf(const TMArrangement& arr, Point<K> centroid);
}

#endif //CARTOCROW_ARRANGEMENT_HELPERS_H
