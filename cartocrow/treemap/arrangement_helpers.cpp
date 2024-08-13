#include "arrangement_helpers.h"

namespace cartocrow::treemap {
Polygon<K> faces_to_polygon(const std::unordered_set<FaceH>& faces) {
	std::optional<TMArrangement::Ccb_halfedge_circulator> start_edge;
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

/// Create an arrangement of a single rectangle. Returns the bounded face.
std::pair<std::shared_ptr<TMArrangement>, FaceH> arrangement_rectangle(const Rectangle<K>& rect) {
	auto arr = std::make_shared<TMArrangement>();

	// Initial square
	Point<K> bl = rect.vertex(Corner::BL);
	Point<K> br = rect.vertex(Corner::BR);
	Point<K> tr = rect.vertex(Corner::TR);
	Point<K> tl = rect.vertex(Corner::TL);
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

	return {arr, face};
}

FaceConstH getFaceOf(const TMArrangement& arr, Point<K> point) {
	TM_pl pl(arr);
	auto loc = pl.locate(point);
	const TMArrangement::Vertex_const_handle* v;
	const TMArrangement::Halfedge_const_handle* e;
	const TMArrangement::Face_const_handle* f;
	if ((f = boost::get<TMArrangement::Face_const_handle>(&loc))) { // located inside a face
		return *f;
	}
	else {
		throw std::runtime_error("Point does not lie in the interior of a face.");
	}
}
}
