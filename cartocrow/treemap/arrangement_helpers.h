#ifndef CARTOCROW_ARRANGEMENT_HELPERS_H
#define CARTOCROW_ARRANGEMENT_HELPERS_H

#include "treemap_helpers.h"
namespace cartocrow::treemap {
Polygon<K> faces_to_polygon(const std::unordered_set<FaceH>& faces);
}

#endif //CARTOCROW_ARRANGEMENT_HELPERS_H
