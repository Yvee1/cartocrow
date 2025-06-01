#include "convex.h"
#include <CGAL/Arr_walk_along_line_point_location.h>

namespace cartocrow::treemap {
std::pair<Point<K>, Point<K>> slice_polygon_line_helper(const Polygon<K>& poly, const Number<K>& ratio, const Line<K>& line) {
    std::vector<Point<K>> sorted;
	std::copy(poly.vertices_begin(), poly.vertices_end(), std::back_inserter(sorted));
	std::sort(sorted.begin(), sorted.end(), [line](const Point<K>& v1, const Point<K>& v2) {
		          auto comp = line.to_vector().perpendicular(CGAL::NEGATIVE) * (v2 - v1);
		          if (comp < 0) {
			          return true;
		          } else if (comp == 0) {
						return line.to_vector() * (v2 - v1) > 0;
		          } else {
			          return false;
		          }
	          }); // todo: check colinearity with line

	// Split poly at extreme points along line.
	// We walk across both parts. These are the two indices of the vertices of poly.
	auto prev = sorted[0];
	int p = std::distance(poly.vertices_begin(), std::find(poly.vertices_begin(), poly.vertices_end(), prev));
	int m = p;

	int n = sorted.size();

	auto proj = [poly, line](const Point<K>& p, int a, int b) {
	  Segment<K> seg(poly.vertex(a), poly.vertex(b));
	  if (squared_distance(seg.start(), p) < M_EPSILON || (seg.start() - p).direction() == line.direction() || (p - seg.start()).direction() == line.direction()) {
		  return seg.start();
	  } else if (squared_distance(seg.end(), p) < M_EPSILON || (seg.end() - p).direction() == line.direction() || (p - seg.end()).direction() == line.direction()) {
		  return seg.end();
	  } {
		  auto inter = intersection(seg.supporting_line(), Line<K>(p, line.direction()));
		  return *boost::get<Point<K>>(&*inter);
	  }
	};

	Number<K> sum = 0;
	Number<K> area = poly.area();

	for (int i = 1; i < sorted.size(); i++) {
		const auto& v = sorted[i];
		Point<K> p1 = proj(prev, p, (p + 1) % n);
		Point<K> p2 = proj(prev, m, (m - 1 + n) % n);
		prev = v;
		Point<K> p3, p4;
		if (v == poly.vertex((p + 1) % n)) {
			p4 = v;
			p3 = proj(p4, m, (m - 1 + n) % n);
			p = (p + 1) % n;
		} else {
			p3 = v;
			p4 = proj(p3, p, (p + 1) % n);
			m = (m - 1 + n) % n;
		}
		if (v == p1 || v == p2) continue;
		if (CGAL::collinear(p1, p2, p3) && CGAL::collinear(p2, p3, p4)) {
			continue;
		}
		std::vector<Point<K>> pts;
		if (p1 == p2) {
			pts.push_back(p1);
		} else {
			pts.push_back(p1);
			pts.push_back(p2);
		}
		if (p3 == p4) {
			pts.push_back(p4);
		} else {
			pts.push_back(p3);
			pts.push_back(p4);
		}
		auto trap = Polygon<K>(pts.begin(), pts.end());
		if (!trap.is_simple()) {
			throw std::runtime_error("Incorrect trapezium.");
		}
		sum += abs(trap.area());

		if (sum >= ratio * area) {
			auto a = sum - ratio * area;
			auto side_2 = sqrt(squared_distance(p1, p2));
			auto side_1 = sqrt(squared_distance(p3, p4));
			auto height = abs(sqrt((line.projection(p1) - line.projection(p3)).squared_length()));
			Number<K> d;
			if (abs(side_1 - side_2) < M_EPSILON) {
				d = a / side_1;
			} else {
				d = height * (sqrt(side_1 * side_1 + (2 * a * (side_2 - side_1)) / height) - side_1) / (side_2 - side_1);
			}
			Vector<K> v = line.direction().perpendicular(CGAL::NEGATIVE).vector();
			Line<K> l(p3 + d * v / sqrt(v.squared_length()), line.direction());
			auto inter1 = intersection(l, Segment<K>(p2, p3).supporting_line());
			auto inter2 = intersection(l, Segment<K>(p1, p4).supporting_line()); // Weird. Without supporting_line() the intersection between 1/sqrt(2) (x + y) - 91.92... and (100, 70), (100, 0) isn't detected.
			Point<K> inter1_pt = *boost::get<Point<K>>(&*inter1);
			Point<K> inter2_pt = *boost::get<Point<K>>(&*inter2);
			return {inter1_pt, inter2_pt};
		}
	}

	throw std::runtime_error("Impossible");
}

std::pair<FaceH, FaceH> slice_polygon_dir(std::shared_ptr<TMArrangement> arr, FaceH& face, const Number<K>& ratio, const Direction<K>& dir) {
	auto poly = face_to_polygon(face);

	auto l1 = Line<K>(CGAL::ORIGIN, dir);
	auto l2 = Line<K>(CGAL::ORIGIN, -dir);

	// todo: pick best of l1 and l2.
	// (1) least number of non-axis-parallel edges
	// (2) lowest aspect ratio
	auto [q1, q2] = slice_polygon_line_helper(poly, ratio, l1);

	TMArrangement::Halfedge_handle closestEdge1;
	TMArrangement::Halfedge_handle closestEdge2;
	std::optional<Number<K>> minDist1 = std::nullopt;
	std::optional<Number<K>> minDist2 = std::nullopt;
	std::optional<VertexH> v1;
	std::optional<VertexH> v2;
	auto eit = face->outer_ccb();
	do {
		Segment<K> seg = eit->curve();
		if (CGAL::squared_distance(eit->source()->point(), q1) <= M_EPSILON * M_EPSILON) {
			v1 = eit->source();
		}
		if (CGAL::squared_distance(eit->source()->point(), q2) <= M_EPSILON * M_EPSILON) {
			v2 = eit->source();
		}
		auto dist1 = CGAL::squared_distance(seg, q1);
		auto dist2 = CGAL::squared_distance(seg, q2);
		if (!minDist1.has_value() || dist1 < *minDist1) {
			minDist1 = dist1;
			closestEdge1 = eit;
		}
		if (!minDist2.has_value() || dist2 < *minDist2) {
			minDist2 = dist2;
			closestEdge2 = eit;
		}
		++eit;
	} while (eit != face->outer_ccb());

	if (!v1.has_value()) {
		TMArrangement::X_monotone_curve_2 c11(closestEdge1->source()->point(), q1);
		TMArrangement::X_monotone_curve_2 c12(q1, closestEdge1->target()->point());
		arr->remove_edge(closestEdge1);
		auto he1 = CGAL::insert_non_intersecting_curve(*arr, c11);
		auto he2 = CGAL::insert_non_intersecting_curve(*arr, c12);
		if (he1->source()->point() == q1) {
			v1 = he1->source();
		} else {
			v1 = he1->target();
		}
	}
	if (!v2.has_value()) {
		TMArrangement::X_monotone_curve_2 c21(closestEdge2->source()->point(), q2);
		TMArrangement::X_monotone_curve_2 c22(q2, closestEdge2->target()->point());
		arr->remove_edge(closestEdge2);
		auto he1 = CGAL::insert_non_intersecting_curve(*arr, c21);
		auto he2 = CGAL::insert_non_intersecting_curve(*arr, c22);
		if (he1->source()->point() == q2) {
			v2 = he1->source();
		} else {
			v2 = he1->target();
		}
	}

	Segment<K> cut((*v1)->point(), (*v2)->point());
	auto cutH = arr->insert_at_vertices(cut, *v1, *v2);
	FaceH h1;
	FaceH h2;

	auto r1 = abs(face_to_polygon(cutH->face()).area() / poly.area() - ratio);
	auto r2 = abs(face_to_polygon(cutH->twin()->face()).area() / poly.area() - ratio);

	if (r1 < r2) {
		h1 = cutH->face();
		h2 = cutH->twin()->face();
	} else {
		h2 = cutH->face();
		h1 = cutH->twin()->face();
	}

	return {h1, h2};
}

std::pair<FaceH, FaceH> slice_polygon_straight(std::shared_ptr<TMArrangement> arr, FaceH& face, const Number<K>& ratio) {
	std::vector<Point<K>> pts;
	auto circ = face->outer_ccb();
	auto curr = circ;
	do {
		pts.push_back(curr->source()->point());
	} while (++curr != circ);
	Polygon<K> poly_test(pts.begin(), pts.end());
//	remove_collinear_vertices(poly);
	auto poly = face_to_polygon(face);
	auto w = poly.right_vertex()->x() - poly.left_vertex()->x();
	auto h = poly.top_vertex()->y() - poly.bottom_vertex()->y();

	Direction<K> dir = w >= h ? Direction<K>(0, 1) : Direction<K>(1, 0);

	return slice_polygon_dir(arr, face, ratio, dir);
}

Direction<K> fresh_direction(const Polygon<K>& poly) {
	int n = poly.size();
	Direction<K> xAxis(1, 0);
	std::vector<Direction<K>> directions({{1, 0}, {0, 1}, {-1, 0}});

	for (int i = 0; i < n; i++) {
		auto v = poly.vertex((i + 1) % n) - poly.vertex(i);
		if (v.y() < 0) {
			v = v * -1;
		}
		directions.emplace_back(v);
	}

	// There is a total order on directions.
	// We compare the angles between the positive x-axis and the directions in counterclockwise order.
	std::sort(directions.begin(), directions.end());

	Vector<K> freshest;
	Number<K> half_cos = 42;

	for (int i = 0; i < directions.size() - 1; i++) {
		auto v1 = directions.at(i).vector();
		auto v2 = directions.at(i + 1).vector();

		v1 /= sqrt(v1.squared_length());
		v2 /= sqrt(v2.squared_length());

		auto v = v1 + v2;
		v /= sqrt(v.squared_length());

		auto h_cos = v * v1;

		if (h_cos < half_cos) {
			half_cos = h_cos;
			freshest = v;
		}
	}

	return freshest.direction();
}

std::pair<FaceH, FaceH> slice_polygon_new_dir(std::shared_ptr<TMArrangement>& arr, FaceH& face, const Number<K>& ratio) {
	auto poly = face_to_polygon(face);
	Direction<K> dir = fresh_direction(poly);

	return slice_polygon_dir(arr, face, ratio, dir);
}

void recurse_convex(const NPD& tree, std::shared_ptr<TMArrangement> arr, FaceH face,
                    std::unordered_map<NPD, std::pair<VertexH, Direction<K>>>& leaf_regions) {
	face_to_polygon(face);
	if (tree->is_leaf()) {
		auto edge = *face->outer_ccb();
		auto vertex = edge.target();
		auto dir = (edge.target()->point() - edge.source()->point()).direction();
		leaf_regions[tree] = {vertex, dir};
		return;
	} else {
		auto v1 = *largest_child(tree, npd_w);

		FaceH h1;
		FaceH h2;
//		 todo: Why not: if (v1->value.weight > tree->value.weight * 2 / 3)?
		if (v1->value.depth == tree->value.depth + 1) {
//		if (v1->value.weight > tree->value.weight * 2 / 3) {
			// Case 1
			// introduce new cutting line
			std::tie(h1, h2) = slice_polygon_new_dir(arr, face, v1->value.weight / tree->value.weight);
		} else {
//			assert(v1->value.depth == tree->value.depth);
			// Case 2
			// do axis parallel cut
			std::tie(h1, h2) = slice_polygon_straight(arr, face, v1->value.weight / tree->value.weight);
		}

		auto v2 = get_other_child(tree, v1);

		auto poly1 = face_to_polygon(h1);
		auto poly2 = face_to_polygon(h2);
		recurse_convex(v1, arr, h1, leaf_regions);
		auto edge = *h2->outer_ccb();
		auto vertex = edge.target();
		auto dir = (edge.target()->point() - edge.source()->point()).direction();
		auto cit_start = vertex->incident_halfedges();
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
		recurse_convex(v2, arr, the_he->face(), leaf_regions);
		return;
	}
}
}
