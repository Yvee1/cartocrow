#ifndef CARTOCROW_ARRANGEMENT_HELPERS_HPP
#define CARTOCROW_ARRANGEMENT_HELPERS_HPP

#include "arrangement_helpers.h"

namespace cartocrow::treemap {
template <class OutputIterator>
void maximalSegments(const TMArrangement& arr, OutputIterator out) {
	std::vector<HalfedgeH> edges;
	for (auto eit = arr.edges_begin(); eit != arr.edges_end(); ++eit) {
		edges.push_back(eit.ptr());
	}

	while (!edges.empty()) {
		std::vector<HalfedgeH> halfedges;
		auto search = [&edges, &halfedges](HalfedgeH e) {
			while (true) {
				edges.erase(std::remove(edges.begin(), edges.end(), e), edges.end());

				auto next = nextOnMaximalSegment(e);
				if (next.has_value()) {
					e = *next;
					halfedges.push_back(e);
				} else {
					break;
				}
			}
		};

		auto initial = *edges.begin();
		search(initial->twin());
		std::reverse(halfedges.begin(), halfedges.end());
		halfedges.push_back(initial);
		search(initial);
		Segment<K> seg(halfedges.front()->source()->point(), halfedges.back()->target()->point());
		*out++ = MaximalSegment(seg, halfedges);
	}
}
}

#endif //CARTOCROW_ARRANGEMENT_HELPERS_HPP
