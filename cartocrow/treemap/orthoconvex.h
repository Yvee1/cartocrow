#ifndef CARTOCROW_ORTHOCONVEX_H
#define CARTOCROW_ORTHOCONVEX_H

#include "treemap_helpers.h"
#include "treemap.h"

namespace cartocrow::treemap {
void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, FaceH& face,
					 Corner corner, std::unordered_map<NPV, FaceH>& leaf_regions);

Treemap build_treemap(const NPV& tree, Corner corner = Corner::BR);
}

#endif //CARTOCROW_ORTHOCONVEX_H
