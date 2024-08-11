#include "convex.h"
#include <CGAL/Arr_walk_along_line_point_location.h>

namespace cartocrow::treemap {
std::pair<Point<K>, Point<K>> slice_polygon_line_helper(const Polygon<K>& poly, const Number<K>& ratio, const Line<K>& line) {
//	for (auto vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit) {
//		auto v = *vit;
//
//	}
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
//	          return comp;
	          }); // todo: check colinearity with line

//	for (const auto& pt : sorted) {
//		std::cout << pt << "  " << squared_distance(line, pt) << std::endl;
//	}
//
//	for (int i = 0; i < sorted.size() - 1; i++) {
//		std::cout << CGAL::compare_signed_distance_to_line(line, sorted[i], sorted[i+1]) << std::endl;
//	}
//
//	std::cout << "---" << std::endl;
	// Split poly at extreme points along line.
	// We walk across both parts. These are the two indices of the vertices of poly.
	auto prev = sorted[0];
	int p = std::distance(poly.vertices_begin(), std::find(poly.vertices_begin(), poly.vertices_end(), prev));
	int m = p;

	int n = sorted.size();

	auto proj = [poly, line](const Point<K>& p, int a, int b) {
	  Segment<K> seg(poly.vertex(a), poly.vertex(b));
//	  auto supp_line = seg.supporting_line();
	  if (seg.start() == p || (seg.start() - p).direction() == line.direction() || (p - seg.start()).direction() == line.direction()) {
		  return seg.start();
	  } else if (seg.end() == p || (seg.end() - p).direction() == line.direction() || (p - seg.end()).direction() == line.direction()) {
		  return seg.end();
	  } {
//		  return seg.supporting_line().projection(p);
		  auto inter = intersection(seg.supporting_line(), Line<K>(p, line.direction()));
		  return *boost::get<Point<K>>(&*inter);
	  }
	};

	Number<K> sum = 0;
	Number<K> area = poly.area();

	Number<K> cut_position;

	for (int i = 1; i < sorted.size(); i++) {
		const auto& v = sorted[i];
		Point<K> p1 = proj(prev, p, (p + 1) % n);
		Point<K> p2 = proj(prev, m, (m - 1 + n) % n);
		prev = v;
		if (v == p1 || v == p2) continue;
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
//		sum += (p1.x() - p2.x() + p3.x() - p4.x()) * (p3.y() - p1.y()) / 2;

		if (sum >= ratio * area) {
			auto a = sum - ratio * area;
			auto side_2 = sqrt(squared_distance(p1, p2));
			auto side_1 = sqrt(squared_distance(p3, p4));
			auto height = abs(sqrt((line.projection(p1) - line.projection(p3)).squared_length()));
			Number<K> d;
			if (side_1 == side_2) {
				d = a / side_1;
			} else {
				d = height * (sqrt(side_1 * side_1 + (2 * a * (side_2 - side_1)) / height) - side_1) / (side_2 - side_1);
			}
			Vector<K> v = line.direction().perpendicular(CGAL::NEGATIVE).vector();
			Line<K> l(p3 + d * v / sqrt(v.squared_length()), line.direction());
//			std::cout << "a: " << CGAL::to_double(a) << std::endl;
//			std::cout << "d: " << CGAL::to_double(d) << std::endl;
//			std::cout << "v: " << approximate(v) << std::endl;
//			std::cout << "l: " << approximate(l) << std::endl;
			auto inter1 = intersection(l, Segment<K>(p2, p3).supporting_line());
			auto inter2 = intersection(l, Segment<K>(p1, p4).supporting_line()); // Weird. Without supporting_line() the intersection between 1/sqrt(2) (x + y) - 91.92... and (100, 70), (100, 0) isn't detected.
//			std::cout << l << std::endl;
//			auto test = intersection(l, Segment<K>(p1, p4).supporting_line());
//			std::cout << Segment<K>(p1, p4) << std::endl;
//			if (test) {
//					const Point<K>* p = boost::get<Point<K>>(&*test);
//					std::cout << *p << std::endl;
//			}
			Point<K> inter1_pt = *boost::get<Point<K>>(&*inter1);
			Point<K> inter2_pt = *boost::get<Point<K>>(&*inter2);
			return {inter1_pt, inter2_pt};
		}
	}

	throw std::runtime_error("Impossible");
}

TMArrangement::Vertex_handle approx_insert(TMArrangement& arr, const Point<K>& point) {
	TM_pl pl(arr);
	auto loc = pl.locate(point);
	const TMArrangement::Vertex_const_handle* v;
	const TMArrangement::Halfedge_const_handle* e;
	const TMArrangement::Face_const_handle* f;
	if ((f = boost::get<TMArrangement::Face_const_handle>(&loc))) { // located inside a face
//		arr.non_const_handle(*f)->outer_ccb();
		auto poly = face_to_polygon(arr.non_const_handle(*f));
		std::optional<Number<K>> min_dist;
		Point<K> closest;
		for (auto eit = poly.edges_begin(); eit != poly.edges_end(); eit++) {
			auto seg = *eit;
			auto proj = seg.supporting_line().projection(point);
			auto dist = (proj - point).squared_length();
			if (!min_dist.has_value() || dist < min_dist) {
				min_dist = dist;
				closest = proj;
			}
		}
		return CGAL::insert_point(arr, closest);
	}
	else if ((e = boost::get<TMArrangement::Halfedge_const_handle>(&loc))) { // located on an edge
		auto edge = *e;
		Segment<K> seg(edge->source()->point(), edge->target()->point());
		return CGAL::insert_point(arr, seg.supporting_line().projection(point));
	}
	else if ((v = boost::get<TMArrangement::Vertex_const_handle>(&loc))) { // located on a vertex
		return arr.non_const_handle(*v);
	}
	else {
		throw std::runtime_error("Impossible");
	}
}

std::pair<FaceH, FaceH> slice_polygon_dir(TMArrangement& arr, FaceH& face, const Number<K>& ratio, const Direction<K>& dir) {
	auto poly = face_to_polygon(face);

	auto l1 = Line<K>(CGAL::ORIGIN, dir);
	auto l2 = Line<K>(CGAL::ORIGIN, -dir);

	// todo: pick best of l1 and l2.
	// (1) least number of non-axis-parallel edges
	// (2) lowest aspect ratio
	auto [q1, q2] = slice_polygon_line_helper(poly, ratio, l1);
	Segment<K> cut(q1, q2);


	auto q1H = approx_insert(arr, q1);
	auto q2H = approx_insert(arr, q2);
	auto cutH = arr.insert_at_vertices(cut, q1H, q2H);
	FaceH h1;
	FaceH h2;
	if (abs(face_to_polygon(cutH->face()).area() / poly.area() - ratio) < (face_to_polygon(cutH->twin()->face()).area() / poly.area() - ratio)) {
		h1 = cutH->face();
		h2 = cutH->twin()->face();
	} else {
		h2 = cutH->face();
		h1 = cutH->twin()->face();
	}

	// todo: return h1 and h2 appropriately
	return {h1, h2};
}

std::pair<FaceH, FaceH> slice_polygon_straight(TMArrangement& arr, FaceH& face, const Number<K>& ratio) {
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
//			v = Vector<K>(v.x(), -v.y());
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

std::pair<FaceH, FaceH> slice_polygon_new_dir(TMArrangement& arr, FaceH& face, const Number<K>& ratio) {
	auto poly = face_to_polygon(face);
	Direction<K> dir = fresh_direction(poly);

	return slice_polygon_dir(arr, face, ratio, dir);
}

void recurse_convex(const NPD& tree, TMArrangement& arr, FaceH& face,
                    std::unordered_map<NPD, FaceH>& leaf_regions) {
	if (tree->is_leaf()) {
		leaf_regions[tree] = face;
		return;
	} else {
		auto v1 = *largest_child(tree, npd_w);

		FaceH h1;
		FaceH h2;
		// todo: Why not: if (v1->value.weight > tree->value.weight * 2 / 3)?
		if (v1->value.depth == tree->value.depth + 1) {
			// Case 1
			// introduce new cutting line
			std::tie(h1, h2) = slice_polygon_new_dir(arr, face, v1->value.weight / tree->value.weight);
		} else {
			assert(v1->value.depth == tree->value.depth);
			// Case 2
			// do axis parallel cut
			std::tie(h1, h2) = slice_polygon_straight(arr, face, v1->value.weight / tree->value.weight);
		}

		auto v2 = get_other_child(tree, v1);

		recurse_convex(v1, arr, h1, leaf_regions);
		recurse_convex(v2, arr, h2, leaf_regions);
		return;
	}
}
}
