#include "treemap_helpers.h"

namespace cartocrow::treemap {
void remove_collinear_vertices(Polygon<K>& polygon) {
	if (polygon.size() <= 3) return;

	bool progress = true;
	while (progress) {
		progress = false;

		auto cit = polygon.vertices_circulator();
		auto curr = cit;
		do {
			auto prev = curr;
			--prev;
			auto next = curr;
			++next;
			if (CGAL::collinear(*prev, *curr, *next)) {
				polygon.erase(curr);
				progress = true;
				break;
			}
		} while (++curr != cit);
	}
}

Polygon<K> face_to_polygon(const Arrangement<K>::Face_const_handle& face) {
	std::vector<Point<K>> pts;
	auto circ = face->outer_ccb();
	auto curr = circ;
	do {
		pts.push_back(curr->source()->point());
	} while (++curr != circ);
	Polygon<K> poly(pts.begin(), pts.end());
	remove_collinear_vertices(poly);
	return poly;
}

//std::pair<NPV, int> label_tree(const NP<Number<K>>& tree, int start) {
//	if (tree->is_leaf()) {
//		Value v{start, tree->value};
//		return {std::make_shared<Node<Value>>(v), start + 1};
//	} else {
//		auto [labeled_left, n] = label_tree(tree->children[0], start);
//		auto [labeled_right, m] = label_tree(tree->children[1], n);
//		auto labeled_node = std::make_shared<Node<Value>>(Value{m, tree->value});
//		labeled_node->add_child(labeled_left);
//		labeled_node->add_child(labeled_right);
//
//		return {labeled_node, m + 1};
//	}
//}

Rectangle<K> face_to_rectangle(const Arrangement<K>::Face_const_handle& face) {
	auto poly = face_to_polygon(face);
	Rectangle<K> rect(poly.vertex(0), poly.vertex(2));
	return rect;
}

NPV range_search(const NPV& tree, const Number<K>& max) {
	auto current = tree;
	while (!current->is_leaf() && current->value > max) {
		auto it = std::max_element(
		    current->children.begin(), current->children.end(),
		    [](const auto& c1, const auto& c2) { return c1->value < c2->value; });
		current = *it;
	}
	return current;
}

NPV get_other_child(const NPV& tree, const NPV& child) {
	if (tree->children.size() != 2) {
		throw std::runtime_error("Precondition violated: tree is not a binary node");
	}

	if (tree->children[0] == child) {
		return tree->children[1];
	} else if (tree->children[1] == child) {
		return tree->children[0];
	} else {
		throw std::runtime_error("Precondition violated: child is not a subtree of tree");
	}
}

void update_weights(NPV& tree) {
	if (!tree->is_leaf()) {
		Number<K> total_weight(0);
		for (const auto& child : tree->children) {
			total_weight += child->value;
		}
		tree->value = total_weight;
	}
	auto p = tree->parent.lock();
	if (p != nullptr) {
		update_weights(p);
	}
}

void replace_child(NPV& tree, const NPV& old_child, const NPV& new_child) {
	int found = -1;
	for (int i = 0; i < tree->children.size(); i++) {
		if (tree->children[i] == old_child) {
			found = i;
		}
	}
	tree->children[found] = new_child;

	update_weights(tree);
}

MarkedNPV split_off_subtree(const NPV& tree, const NPV& subtree) {
	// Cut off the subtree
	auto parent = subtree->parent.lock();
	subtree->parent = std::weak_ptr<Node<Number<K>>>();

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

std::pair<FaceH, FaceH> slice_rectangle(Arrangement<K>& arr, FaceH& face,
                                        const Number<K>& corner_ratio, Corner corner,
                                        std::optional<bool> force_split_dir) {
	auto rect = face_to_rectangle(face);
	auto w = width(rect);
	auto h = height(rect);
	bool vertical = w >= h;
	if (force_split_dir.has_value()) {
		vertical = *force_split_dir;
	}
	auto p1 = rect.vertex(corner);
	auto p2 =
	    vertical && corner % 2 == 1 || !vertical && corner % 2 == 0 ? rect.vertex((corner + 3) % 4) : rect.vertex((corner + 1) % 4);
	auto q1 = p1 + (p2 - p1) * corner_ratio;
	auto q2 = vertical ? Point<K>(q1.x(), rect.ymin() + (rect.ymax() - q1.y()))
	                   : Point<K>(rect.xmin() + (rect.xmax() - q1.x()), q1.y());

	Segment<K> cut(q1, q2);
	auto q1H = CGAL::insert_point(arr, q1);
	auto q2H = CGAL::insert_point(arr, q2);
	auto cutH = arr.insert_at_vertices(cut, q1H, q2H);

	FaceH cornerH;
	FaceH oppositeH;
	if (vertical && corner % 2 == 1 || !vertical && corner % 2 == 0) {
		cornerH = cutH->twin()->face();
		oppositeH = cutH->face();
	} else {
		cornerH = cutH->face();
		oppositeH = cutH->twin()->face();
	}

	return {cornerH, oppositeH};
}

std::pair<FaceH, FaceH> create_notch(Arrangement<K>& arr, const Rectangle<K>& rect,
                                     const Number<K>& notch_ratio, Corner corner) {
	auto next_corner = static_cast<Corner>((corner + 1) % 4);
	auto prev_corner = static_cast<Corner>((corner + 3) % 4);
	auto cr = rect.vertex(corner);
	auto crn = rect.vertex(next_corner);
	auto crp = rect.vertex(prev_corner);

	auto p1 = cr + (crn - cr) * sqrt(notch_ratio);
	auto p2 = cr + (crp - cr) * sqrt(notch_ratio);
	Point<K> p12;
	if (is_horizontal(side(rect, prev_corner, corner))) {
		p12 = {p2.x(), p1.y()};
	} else {
		p12 = {p1.x(), p2.y()};
	}

	auto p1H = CGAL::insert_point(arr, p1);
	auto p2H = CGAL::insert_point(arr, p2);
	auto p12H = CGAL::insert_point(arr, p12);
	auto s1H = arr.insert_at_vertices({p1, p12}, p1H, p12H);
	auto s2H = arr.insert_at_vertices({p12, p2}, p12H, p2H);

	FaceH rectH;
	FaceH lH;
	if (s1H->source() == p1H) {
		rectH = s1H->face();
		lH = s1H->twin()->face();
	} else {
		rectH = s1H->twin()->face();
		lH = s1H->face();
	}
	return {lH, rectH};
}

std::pair<FaceH, FaceH> slice_L_rectangle(Arrangement<K>& arr, FaceH& face,
                                          const Number<K>& corner_ratio, Corner corner) {
	auto rect = face_to_rectangle(face);
	return create_notch(arr, rect, 1 - corner_ratio, opposite(corner));
}

std::pair<FaceH, std::pair<FaceH, FaceH>>
slice_S_rectangle(Arrangement<K>& arr, FaceH& face, const Number<K>& corner_ratio,
                  const Number<K>& opposite_ratio, Corner corner) {
	auto rect = face_to_rectangle(face);
	auto [_, cornerH] = create_notch(arr, rect, corner_ratio, corner);
	auto [sH, oppositeH] = create_notch(arr, rect, opposite_ratio, opposite(corner));
	return {sH, {cornerH, oppositeH}};
}

std::pair<NPV, NPV> ancestor_search(const NPV& node, const Number<K>& max_weight) {
	NPV mu_hat = node;
	NPV mu_star = node->parent.lock();

	while (mu_star->value < max_weight) {
		mu_hat = mu_star;
		mu_star = mu_hat->parent.lock();
	}

	return {mu_hat, mu_star};
}

NPV largest_leaf(NPV node) {
	if (node->is_leaf())
		return node;
	NPV largest;
	Number<K> max_weight = -1;
	for (const auto& child : node->children) {
		auto leaf = largest_leaf(child);
		if (leaf->value > max_weight) {
			max_weight = leaf->value;
			largest = leaf;
		}
	}
	return largest;
}

NPV copy_tree(const NPV& tree, std::unordered_map<NPV, NPV>& copy_to_tree) {
	if (tree->is_leaf()) {
		auto new_node = std::make_shared<Node<Number<K>>>(tree->value);
		copy_to_tree[new_node] = tree;
		return new_node;
	} else {
		auto node = std::make_shared<Node<Number<K>>>(tree->value);
		for (const auto& child : tree->children) {
			auto copy = copy_tree(child, copy_to_tree);
			node->add_child(copy);
		}
		copy_to_tree[node] = tree;
		return node;
	}
}
}