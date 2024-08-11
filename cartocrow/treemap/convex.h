#ifndef CARTOCROW_CONVEX_H
#define CARTOCROW_CONVEX_H

#include "treemap_helpers.h"
#include "treemap.h"

namespace cartocrow::treemap {
void recurse_convex(const NPD& tree, TMArrangement& arr, FaceH& face,
                    std::unordered_map<NPD, FaceH>& leaf_regions);

template <class V>
Treemap<V> convex_treemap(NP<V>& tree, const Rectangle<K>& rect, NodeWeight<V> w) {
	auto [arr, face] = arrangement_rectangle(rect);

	std::unordered_map<NPD, FaceH> leaf_regions;
	std::unordered_map<NPD, NP<V>> copy_to_tree;
	NPD copy = convert_to_binary_npv_balanced(tree, copy_to_tree, w);
	recurse_convex(copy, *arr, face, leaf_regions);

	std::unordered_map<NP<V>, FaceH> original_leaf_regions;
	for (const auto& leaf_region : leaf_regions) {
		if (copy_to_tree.contains(leaf_region.first)) {
			original_leaf_regions[copy_to_tree.at(leaf_region.first)] = leaf_region.second;
		}
	}
	return {tree, arr, original_leaf_regions, rect};
}

typedef CGAL::Arr_walk_along_line_point_location<TMArrangement> TM_pl;
}

#endif //CARTOCROW_CONVEX_H
