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

	std::string filename = "/home/steven/Documents/cartocrow/data/test.tree";
	std::fstream fin(filename);
	std::string input;
	if (fin) {
		using Iterator = std::istreambuf_iterator<char>;
		input.assign(Iterator(fin), Iterator());
	}
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

	recurse_treemap(tree, tree, arr, face);
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

std::pair<Arrangement<K>::Face_handle, Arrangement<K>::Face_handle>
slice_rectangle(Arrangement<K>& arr, Arrangement<K>::Face_handle& face, double ratio) {
	auto rect = face_to_rectangle(face);

	Arrangement<K>::Face_handle leftTopH;
	Arrangement<K>::Face_handle rightBotH;
	if (rect.xmax() - rect.xmin() >= rect.ymax() - rect.ymin()) {
		// vertical cut
		// todo: put heavy side on the right.
		Point<K> top(ratio * rect.xmin() + (1 - ratio) * rect.xmax(), rect.ymax());
		Point<K> bot(ratio * rect.xmin() + (1 - ratio) * rect.xmax(), rect.ymin());
		Segment<K> cut(top, bot);
		auto topH = CGAL::insert_point(arr, top);
		auto botH = CGAL::insert_point(arr, bot);
		auto cutH = CGAL::insert_non_intersecting_curve(arr, cut);
		if (cutH->source() == botH) {
			leftTopH = cutH->face();
			rightBotH = cutH->twin()->face();
		} else {
			leftTopH = cutH->twin()->face();
			rightBotH = cutH->face();
		}
	} else {
		// horizontal cut
		Point<K> left(rect.xmin(), ratio * rect.ymin() + (1-ratio) * rect.ymax());
		Point<K> right(rect.xmax(), ratio * rect.ymin() + (1-ratio) * rect.ymax());
		Segment<K> cut(left, right);
		auto leftH = CGAL::insert_point(arr, left);
		auto rightH = CGAL::insert_point(arr, right);
		auto cutH = CGAL::insert_non_intersecting_curve(arr, cut);
		if (cutH->source() == rightH) {
			leftTopH = cutH->face();
			rightBotH = cutH->twin()->face();
		} else {
			leftTopH = cutH->twin()->face();
			rightBotH = cutH->face();
		}
	}
	return {leftTopH, rightBotH};
}

void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, Arrangement<K>::Face_handle& face) {
	if (tree->is_leaf()) return;
	auto weight = tree->value.weight;

	if (marked->value.weight >= weight / 8) {
		auto nu = range_search(marked, weight / 8, 7 * weight / 8);
		auto ratio = nu->value.weight / tree->value.weight;
		if (ratio <= 0.875) {
			// match left/right to the two parts of the tree
			auto [leftTopH, rightBotH] = slice_rectangle(arr, face, ratio);
			auto remainder = split_off_subtree(tree, nu);
			recurse_treemap(remainder.tree, remainder.marked, arr, leftTopH);
			recurse_treemap(nu, nu, arr, rightBotH);
		} else {
			std::cout << "TODO" << std::endl;
		}
	} else {
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