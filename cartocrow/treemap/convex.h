#ifndef CARTOCROW_CONVEX_H
#define CARTOCROW_CONVEX_H

#include "treemap_helpers.h"
#include "treemap.h"

namespace cartocrow::treemap {
void recurse_convex(const NPD& tree, std::shared_ptr<TMArrangement> arr, FaceH face,
                    std::unordered_map<NPD, std::pair<VertexH, Direction<K>>>& leaf_regions);

template <class V>
Treemap<V> convex_treemap(NP<V>& tree, const Rectangle<K>& rect, NodeWeight<V> w) {
	auto [arr, face] = arrangement_rectangle(rect);

	std::unordered_map<NPD, FaceH> leaf_regions;
	std::unordered_map<NPD, std::pair<VertexH, Direction<K>>> leaf_regions_safe;
	std::unordered_map<NPD, NP<V>> copy_to_tree;
	NPD copy = convert_to_binary_npv_balanced(tree, copy_to_tree, w);
	recurse_convex(copy, arr, face, leaf_regions_safe);

	std::cout << "--- All faces ---" << std::endl;
	for (auto fit = arr->faces_begin(); fit != arr->faces_end(); fit++) {
		if (fit->has_outer_ccb()) {
			std::cout << face_to_polygon_coll(fit) << std::endl;
		}
	}
	std::cout << "--- All edges ---" << std::endl;
	for (auto eit = arr->edges_begin(); eit != arr->edges_end(); eit++) {
		std::cout << (eit->source()->point()) << " -> " << (eit->target()->point()) << std::endl;
	}

	for (const auto& leaf_region : leaf_regions_safe) {
		auto [v, dir] = leaf_region.second;
		auto cit_start = v->incident_halfedges();
		auto cit = cit_start;
		HalfedgeH the_he;
		bool found = false;
		do {
			auto he = *cit;
			if ((he.target()->point() - he.source()->point()).direction() == dir) {
				the_he = cit;
				found = true;
				break;
			}
		} while (++cit != cit_start);

		if (!found) {
			throw std::runtime_error("Could not find face from saved vertex.");
		}
		leaf_regions[leaf_region.first] = the_he->face();
	}

	std::unordered_map<NP<V>, FaceH> original_leaf_regions;
	for (const auto& leaf_region : leaf_regions) {
		if (copy_to_tree.contains(leaf_region.first)) {
			original_leaf_regions[copy_to_tree.at(leaf_region.first)] = leaf_region.second;
		}
	}
	return {tree, arr, original_leaf_regions, rect};
}
}

#endif //CARTOCROW_CONVEX_H
