#ifndef CARTOCROW_ORTHOCONVEX_H
#define CARTOCROW_ORTHOCONVEX_H

#include "treemap_helpers.h"
#include "treemap.h"

namespace cartocrow::treemap {
void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, FaceH& face,
					 Corner corner, std::unordered_map<NPV, FaceH>& leaf_regions);

Treemap<Number<K>> build_treemap(NP<Number<K>>& tree, Corner corner = Corner::BR);

template <class V>
Treemap<V> build_treemap(NP<V>& tree, std::function<Number<K>(NP<V>)> w, Corner corner = Corner::BR) {
	auto arr = std::make_shared<Arrangement<K>>();

	Rectangle<K> rect({0, 0}, {100, 100});

	// Initial square
	Point<K> bl(0, 0);
	Point<K> br(100, 0);
	Point<K> tr(100, 100);
	Point<K> tl(0, 100);
	Segment<K> bot(bl, br);
	Segment<K> right(br, tr);
	Segment<K> top(tl, tr);
	Segment<K> left(bl, tl);
	CGAL::insert_non_intersecting_curve(*arr, bot);
	CGAL::insert_non_intersecting_curve(*arr, right);
	CGAL::insert_non_intersecting_curve(*arr, top);
	auto handle = CGAL::insert_non_intersecting_curve(*arr, left);

	FaceH face;
	if (!handle->face()->is_unbounded()) {
		face = handle->face();
	} else {
		face = handle->twin()->face();
	}

	std::unordered_map<NPV, FaceH> leaf_regions;
	std::unordered_map<NPV, NP<V>> copy_to_tree;
	NPV copy = convert_to_binary_npv(tree, copy_to_tree, w);
	recurse_treemap(copy, copy, *arr, face, corner, leaf_regions);

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
