#include "treemap_demo.h"
#include <QApplication>
#include <utility>
#include "parse_tree.h"

TreemapDemo::TreemapDemo() {
	setWindowTitle("Treemap");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

//	std::string filename = "/home/steven/Documents/cartocrow/data/test.tree";
//	std::fstream fin(filename);
//	std::string input;
//	if (fin) {
//		using Iterator = std::istreambuf_iterator<char>;
//		input.assign(Iterator(fin), Iterator());
//	}
	auto input = "(((0.1)(0.1))(2))";
	auto node = parse_tree(input);
	std::shared_ptr<Node<Value>> root = label_tree(node).first;

	auto arr = build_treemap(root);
	auto tmp  = std::make_shared<TreemapPainting>(arr);
	m_renderer->addPainting(tmp, "Treemap");
}

std::pair<NPV, int> label_tree(const NP<double>& tree, int start) {
	if (tree->is_leaf()) {
		Value v{start, tree->value};
		return { std::make_shared<Node<Value>>(v), start + 1 };
	} else {
		auto [labeled_left, n] = label_tree(tree->children[0], start);
		auto [labeled_right, m] = label_tree(tree->children[1], n);
		auto labeled_node = std::make_shared<Node<Value>>(Value{m, tree->value});
		labeled_node->add_child(labeled_left);
		labeled_node->add_child(labeled_right);

		return {labeled_node, m+1};
	}
}

Arrangement<K> build_treemap(const NPV& tree) {
	Arrangement<K> arr;

	// Initial square
	Point<K> bl(0, 0);
	Point<K> br(100, 0);
	Point<K> tr(100, 100);
	Point<K> tl(0, 100);
	Segment<K> bot(bl, br);
	Segment<K> right(br, tr);
	Segment<K> top(tl, tr);
	Segment<K> left(bl, tl);
	CGAL::insert_non_intersecting_curve(arr, bot);
	CGAL::insert_non_intersecting_curve(arr, right);
	CGAL::insert_non_intersecting_curve(arr, top);
	auto handle = CGAL::insert_non_intersecting_curve(arr, left);

	Arrangement<K>::Face_handle face;
	if (!handle->face()->is_unbounded()) {
		face = handle->face();
	} else {
		face = handle->twin()->face();
	}

	recurse_treemap(tree, tree, arr, face, Corner::BR);
	return arr;
}

Polygon<K> face_to_polygon(const Arrangement<K>::Face_const_handle& face) {
	std::vector<Point<K>> pts;
	auto circ = face->outer_ccb();
	auto curr = circ;
	do {
		pts.push_back(curr->source()->point());
	} while (++curr != circ);
	Polygon<K> poly(pts.begin(), pts.end());
	return poly;
}

Rectangle<K> face_to_rectangle(const Arrangement<K>::Face_const_handle& face) {
	auto poly = face_to_polygon(face);
	Rectangle<K> rect(poly.vertex(0), poly.vertex(2));
	return rect;
}

NPV range_search(const NPV& tree, double min, double max) {
	auto current = tree;
	while (!current->is_leaf() && current->value.weight > tree->value.weight * 0.875) {
		auto it = std::max_element(current->children.begin(), current->children.end(), [](const auto& c1, const auto& c2) { return c1->value.weight < c2->value.weight; });
		current = *it;
	}
	return current;
}

NPV get_other_child(const NPV& tree, const NPV& child) {
	if (tree->children.size() != 2) {
		throw std::runtime_error("Precondition violated: tree is not a binary node");
	}

	if (tree->children[0]->value.label == child->value.label) {
		return tree->children[1];
	} else if (tree->children[1]->value.label == child->value.label) {
		return tree->children[0];
	} else {
		throw std::runtime_error("Precondition violated: child is not a subtree of tree");
	}
}

void replace_child(NPV& tree, const NPV& old_child, const NPV& new_child) {
	int found = -1;
	for (int i = 0; i < tree->children.size(); i++) {
		if (tree->children[i]->value.label == old_child->value.label) {
			found = i;
		}
	}
	tree->children[found] = new_child;
}

MarkedNPV split_off_subtree(const NPV& tree, const NPV& subtree) {
	// Cut off the subtree
	auto parent = subtree->parent.lock();
	subtree->parent = std::weak_ptr<Node<Value>>();

	// Find sibling
	auto sibling = get_other_child(parent, subtree);

	// Contract
	auto grandparent = parent->parent.lock();
	sibling->parent = grandparent;

	if (grandparent != nullptr) {
		replace_child(grandparent, parent, sibling);
		return {tree, sibling};
	} else {
		return {sibling, sibling};
	}
}

auto width(const Rectangle<K>& rect) {
	return rect.xmax() - rect.xmin();
}

auto height(const Rectangle<K>& rect) {
	return rect.ymax() - rect.ymin();
}

auto dimension(const Rectangle<K>& rect, int i) {
	if (i == 0) {
		return width(rect);
	} else if (i == 1) {
		return height(rect);
	} else {
		throw std::runtime_error("Dimension i is not 0 or 1");
	}
}

Corner corner(const Rectangle<K>& rect, const Side& side1, const Side& side2) {
	if (side1 > side2) return corner(rect, side2, side1);
	int dist = side2 - side1;
	if (dist == 1) {
		return static_cast<Corner>(side1);
	} else if (dist == 3) {
		return static_cast<Corner>(side2);
	} else {
		throw std::runtime_error("Sides are not adjacent");
	}
}

Side side(const Rectangle<K>& rect, Corner corner1, Corner corner2) {
	if (corner1 > corner2) return side(rect, corner2, corner1);
	int dist = corner2 - corner1;
	if (dist == 1) {
		return static_cast<Side>(corner2);
	} else if (dist == 3) {
		return static_cast<Side>(corner1);
	} else {
		throw std::runtime_error("Corners are not adjacent");
	}
}

bool is_horizontal(Side side) {
	return side % 2 == 1;
}

std::pair<Arrangement<K>::Face_handle, Arrangement<K>::Face_handle>
slice_rectangle(Arrangement<K>& arr, Arrangement<K>::Face_handle& face, double corner_ratio, Corner corner, std::optional<bool> force_split_dir = std::nullopt) {
	auto rect = face_to_rectangle(face);
	auto w = width(rect);
	auto h = height(rect);
	bool vertical = w >= h;
	if (force_split_dir.has_value()) {
		vertical = *force_split_dir;
	}
	auto p1 = rect.vertex(corner);
	auto p2 = vertical && corner % 2 == 1 ? rect.vertex((corner + 3) % 4) : rect.vertex((corner + 1) % 4);
	auto q1 = p1 + (p2 - p1) * corner_ratio;
	auto q2 = vertical ? Point<K>(q1.x(), rect.ymin() + (rect.ymax() - q1.y())) : Point<K>(rect.xmin() + (rect.xmax() - q1.x()), q1.y());

	Segment<K> cut(q1, q2);
	auto q1H = CGAL::insert_point(arr, q1);
	auto q2H = CGAL::insert_point(arr, q2);
	// There seems to be no way to provide leftH as the vertex to insert at.
	// CGAL will now have to locate the vertex, which can be somewhat expensive in large treemaps.
	auto cutH = CGAL::insert_non_intersecting_curve(arr, cut);

	Arrangement<K>::Face_handle cornerH;
	Arrangement<K>::Face_handle oppositeH;
	if (cutH->source() == q1H) {
		cornerH = cutH->face();
		oppositeH = cutH->twin()->face();
	} else {
		cornerH = cutH->twin()->face();
		oppositeH = cutH->face();
	}

	return {cornerH, oppositeH};
}

std::pair<Arrangement<K>::Face_handle, Arrangement<K>::Face_handle>
create_notch(Arrangement<K>& arr, const Rectangle<K>& rect, double notch_ratio, Corner corner) {
	auto next_corner = static_cast<Corner>((corner + 1) % 4);
	auto prev_corner = static_cast<Corner>((corner + 3) % 4);
	auto cr = rect.vertex(corner);
	auto crn = rect.vertex(next_corner);
	auto crp = rect.vertex(prev_corner);

	auto p1 = cr + (crn - cr) * notch_ratio;
	auto p2 = cr + (crp - cr) * notch_ratio;
	Point<K> p12;
	if (is_horizontal(side(rect, prev_corner, corner))) {
		p12 = {p2.x(), p1.y()};
	} else {
		p12 = {p1.x(), p2.y()};
	}

	auto p1H = CGAL::insert_point(arr, p1);
	auto p2H = CGAL::insert_point(arr, p2);
	auto p12H = CGAL::insert_point(arr, p12);
	auto s1H = CGAL::insert_non_intersecting_curve(arr, {p1, p12});
	auto s2H = CGAL::insert_non_intersecting_curve(arr, {p12, p2});

	Arrangement<K>::Face_handle rectH;
	Arrangement<K>::Face_handle lH;
	if (s1H->source() == p1H) {
		rectH = s1H->face();
		lH = s1H->twin()->face();
	} else {
		rectH = s1H->twin()->face();
		lH = s1H->face();
	}
	return {lH, rectH};
}

std::pair<Arrangement<K>::Face_handle, Arrangement<K>::Face_handle>
slice_L_rectangle(Arrangement<K>& arr, Arrangement<K>::Face_handle& face, double corner_ratio, Corner corner) {
	auto rect = face_to_rectangle(face);
    return create_notch(arr, rect, 1 - corner_ratio, opposite(corner));
}

std::pair<Arrangement<K>::Face_handle, std::pair<Arrangement<K>::Face_handle, Arrangement<K>::Face_handle>>
slice_S_rectangle(Arrangement<K>& arr, Arrangement<K>::Face_handle& face, double corner_ratio, double opposite_ratio, Corner corner) {
	auto rect = face_to_rectangle(face);
	auto [_,  cornerH] = create_notch(arr, rect, corner_ratio, corner);
	auto [sH, oppositeH] = create_notch(arr, rect, opposite_ratio, opposite(corner));
	return {sH, {cornerH, oppositeH}};
}

std::pair<NPV, NPV>
ancestor_search(const NPV& node, double max_weight) {
	NPV mu_hat = node;
	NPV mu_star = node->parent.lock();

	while (mu_star->value.weight < max_weight) {
		mu_hat = mu_star;
		mu_star = mu_hat->parent.lock();
	}

	return {mu_hat, mu_star};
}

NPV largest_leaf(NPV node) {
	if (node->is_leaf()) return node;
	NPV largest;
	double max_weight = -1;
	for (const auto& child : node->children) {
		auto leaf = largest_leaf(child);
		if (leaf->value.weight > max_weight) {
			max_weight = leaf->value.weight;
			largest = leaf;
		}
	}
	return largest;
}

Corner opposite(Corner corner) {
	return static_cast<Corner>((corner + 2) % 4);
}

void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, Arrangement<K>::Face_handle& face, Corner corner) {
	if (tree->is_leaf()) return;
	auto weight = tree->value.weight;

	if (marked->value.weight >= weight / 8) {
		// Case (a): marked is non-tiny: cut off small or large subtree, or huge leaf.
		auto nu = range_search(marked, weight / 8, 7 * weight / 8);
		auto ratio = nu->value.weight / tree->value.weight;

		Arrangement<K>::Face_handle cornerH;
		Arrangement<K>::Face_handle oppositeH;
		if (ratio <= 0.875) {
			std::tie(cornerH, oppositeH) = slice_rectangle(arr, face, ratio, corner);
		} else {
			std::tie(cornerH, oppositeH) = slice_L_rectangle(arr, face, ratio, corner);
		}
		auto remainder = split_off_subtree(tree, nu);
		recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner);
		recurse_treemap(nu, nu, arr, cornerH, corner);
		return;
	} else {
		auto [mu_hat, mu_star] = ancestor_search(marked, 7 * weight / 8);
		if (mu_hat->value.weight >= weight / 8) {
			// Case (b): mu_hat is small or large: cut off mu_hat
			auto ratio = mu_hat->value.weight / tree->value.weight;
			auto [cornerH, oppositeH] = slice_rectangle(arr, face, ratio, corner);
			auto remainder = split_off_subtree(tree, mu_hat);
			recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner);
			recurse_treemap(mu_hat, marked, arr, cornerH, corner);
			return;
		}
		auto lambda = largest_leaf(get_other_child(mu_star, mu_hat));

		if (lambda->value.weight >= weight / 4) {
			// Case (c): tree has large or huge leaf
			auto tree_minus_lambda = split_off_subtree(tree, lambda);
			if (tree_minus_lambda.tree->value.label == mu_hat->value.label) {
				auto ratio = lambda->value.weight / weight;
				auto [lH, rectH] = slice_L_rectangle(arr, face, ratio, opposite(corner));
				auto remainder = split_off_subtree(tree, lambda);
				recurse_treemap(lambda, lambda, arr, lH, corner);
				recurse_treemap(remainder.tree, marked, arr, rectH, corner);
				return;
			} else {
				auto remainder = split_off_subtree(tree_minus_lambda.tree, mu_hat);
				if (remainder.tree->value.weight <= weight / 8) {
					auto [sH, rectsH] = slice_S_rectangle(arr, face, mu_hat->value.weight / weight, remainder.tree->value.weight / weight, corner);
					auto [cornerH, oppositeH] = rectsH;
					recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner);
					recurse_treemap(lambda, lambda, arr, sH, corner);
					recurse_treemap(mu_hat, marked, arr, cornerH, corner);
					return;
				} else {
					auto [cornerH, oppositeH] = slice_rectangle(arr, face, 1 - remainder.tree->value.weight / weight, corner, true);
					auto [lH, rectH] = slice_L_rectangle(arr, cornerH, lambda->value.weight / weight, opposite(corner));
					recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner);
					recurse_treemap(lambda, lambda, arr, lH, corner);
					recurse_treemap(mu_hat, marked, arr, rectH, corner);
					return;
				}
			}
		}

		// Case (d)
		// todo
		std::cout << "TODO" << std::endl;
	}
}

TreemapPainting::TreemapPainting(Arrangement<K> arr): m_arr(std::move(arr)) {}

void TreemapPainting::paint(GeometryRenderer& renderer) const {
	for (auto fit = m_arr.faces_begin(); fit != m_arr.faces_end(); fit++) {
		if (fit->has_outer_ccb()) {
			auto poly = face_to_polygon(fit);
			renderer.setFill(Color{200, 200, 200});
			renderer.setStroke(Color{0, 0, 0}, 1.0);
			renderer.setMode(GeometryRenderer::fill | GeometryRenderer::stroke);
			renderer.draw(poly);
		}
	}

	for (auto eit = m_arr.edges_begin(); eit != m_arr.edges_end(); eit++) {
		auto edge = *eit;
		Segment<K> seg(edge.source()->point(), edge.target()->point());
		renderer.setStroke(Color{0, 0, 0}, 1.0);
		renderer.setMode(GeometryRenderer::stroke);
		renderer.draw(seg);
	}

	for (auto vit = m_arr.vertices_begin(); vit != m_arr.vertices_end(); vit++) {
		renderer.draw(vit->point());
	}
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	TreemapDemo demo;
	demo.show();
	QApplication::exec();
}