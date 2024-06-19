#include "arrangement_helpers.h"

namespace cartocrow::treemap {
Polygon<K> faces_to_polygon(const std::unordered_set<FaceH>& faces) {
	std::optional<Arrangement<K>::Ccb_halfedge_circulator> start_edge;
	for (const auto& face : faces) {
		auto cit = face->outer_ccb();
		auto current = cit;
		do {
			if (!faces.contains(current->twin()->face())) {
				start_edge = current;
				break;
			}
		} while (++current != cit);

		if (start_edge.has_value()) break;
	}

	if (!start_edge.has_value()) {
		throw std::runtime_error("Could not find boundary edge");
	}
	auto current = *start_edge;
	std::vector<Point<K>> pts;
	do {
		pts.push_back(current->source()->point());
		current = current->next();
		while(faces.contains(current->twin()->face())) {
			current = current->twin()->next();
		}
	} while(current != start_edge);

	return Polygon<K>(pts.begin(), pts.end());
}
}
