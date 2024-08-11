#ifndef CARTOCROW_ORTHOCONVEX_H
#define CARTOCROW_ORTHOCONVEX_H

#include "treemap_helpers.h"
#include "treemap.h"

namespace cartocrow::treemap {
void recurse_orthoconvex(const NPV& tree, const NPV& marked, TMArrangement& arr, FaceH& face,
					 Corner corner, std::unordered_map<NPV, FaceH>& leaf_regions);

Treemap<Number<K>> orthoconvex_treemap(NP<Number<K>>& tree, const Rectangle<K>& rect, Corner corner = Corner::BR);

template <class V>
Treemap<V> orthoconvex_treemap(NP<V>& tree, const Rectangle<K>& rect, NodeWeight<V> w, Corner corner = Corner::BR) {
	auto [arr, face] = arrangement_rectangle(rect);

	std::unordered_map<NPV, FaceH> leaf_regions;
	std::unordered_map<NPV, NP<V>> copy_to_tree;
	NPV copy = convert_to_binary_npv(tree, copy_to_tree, w);
	recurse_orthoconvex(copy, copy, *arr, face, corner, leaf_regions);

	std::unordered_map<NP<V>, FaceH> original_leaf_regions;
	for (const auto& leaf_region : leaf_regions) {
		if (copy_to_tree.contains(leaf_region.first)) {
			original_leaf_regions[copy_to_tree.at(leaf_region.first)] = leaf_region.second;
		}
	}
	return {tree, arr, original_leaf_regions, rect};
}
}

#endif //CARTOCROW_ORTHOCONVEX_H
