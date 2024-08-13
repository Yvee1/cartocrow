#ifndef CARTOCROW_TREEMAP_HELPERS_H
#define CARTOCROW_TREEMAP_HELPERS_H

#include "tree.h"
#include "rectangle_helpers.h"
#include "cartocrow/core/core.h"
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

namespace cartocrow::treemap {
//typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt K;
typedef Inexact K;

struct DepthWeight {
	int depth;
	Number<K> weight;
};

template <class V> using NP = std::shared_ptr<Node<V>>;
template <class V> using CNP = std::shared_ptr<const Node<V>>;
typedef NP<Number<K>> NPV;
typedef NP<DepthWeight> NPD;
typedef CNP<Number<K>> CNPV;

template <class V> using NodeWeight = std::function<Number<K>(NP<V>)>;

template <class V>
std::string tree_to_string(NP<V> node, NodeWeight<V> w) {
	std::stringstream result;
	result << "(";

	if (node->is_leaf()) {
		result << w(node);
	} else {
		for (const auto& child : node->children) {
			result << tree_to_string(child, w);
		}
	}
	result << ")";

	return result.str();
}

extern NodeWeight<Number<K>> npv_w;
extern NodeWeight<DepthWeight> npd_w;

template <class V>
struct MarkedNP {
	NP<V> tree;
	NP<V> marked;
};

typedef  CGAL::Arrangement_2<CGAL::Arr_segment_traits_2<K>> TMArrangement;
typedef TMArrangement::Face_handle FaceH;
typedef TMArrangement::Vertex_handle VertexH;
typedef TMArrangement::Halfedge_handle HalfedgeH;
typedef TMArrangement::Face_const_handle FaceConstH;

/// Removes collinear vertices
Polygon<K> face_to_polygon(const TMArrangement::Face_const_handle& face);
/// Removes collinear vertices
Polygon<K> ccb_to_polygon(TMArrangement::Ccb_halfedge_const_circulator circ);
/// Does not remove collinear vertices
Polygon<K> face_to_polygon_coll(const TMArrangement::Face_const_handle& face);
/// Does not remove collinear vertices
Polygon<K> ccb_to_polygon_coll(TMArrangement::Ccb_halfedge_const_circulator circ);
Rectangle<K> face_to_rectangle(const TMArrangement::Face_const_handle& face);
std::pair<FaceH, FaceH> slice_rectangle(TMArrangement& arr, FaceH& face,
										const Number<K>& corner_ratio, Corner corner,
										std::optional<bool> force_split_dir = std::nullopt);
std::pair<FaceH, FaceH> create_notch(TMArrangement& arr, const Rectangle<K>& rect,
									 const Number<K>& notch_ratio, Corner corner);
std::pair<FaceH, FaceH> slice_L_rectangle(TMArrangement& arr, FaceH& face,
										  const Number<K>& corner_ratio, Corner corner);
std::pair<FaceH, std::pair<FaceH, FaceH>>
slice_S_rectangle(TMArrangement& arr, FaceH& face, const Number<K>& corner_ratio,
				  const Number<K>& opposite_ratio, Corner corner);

template <class V>
NP<V> range_search(const NP<V>& tree, const Number<K>& max, NodeWeight<V> w) {
	auto current = tree;
	while (!current->is_leaf() && current->value > max) {
		auto it = std::max_element(
		    current->children.begin(), current->children.end(),
		    [w](const auto& c1, const auto& c2) { return w(c1) < w(c2); });
		current = *it;
	}
	return current;
}

template <class V>
NP<V> get_other_child(const NP<V>& tree, const NP<V>& child) {
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

template <class V>
void update_weights(NP<V>& tree, NodeWeight<V> w) {
	if (!tree->is_leaf()) {
		Number<K> total_weight(0);
		for (const auto& child : tree->children) {
			total_weight += w(child);
		}
		tree->value = total_weight;
	}
	auto p = tree->parent.lock();
	if (p != nullptr) {
		update_weights(p, w);
	}
}

template <class V>
void replace_child(NP<V>& tree, const NP<V>& old_child, const NP<V>& new_child, NodeWeight<V> w) {
	int found = -1;
	for (int i = 0; i < tree->children.size(); i++) {
		if (tree->children[i] == old_child) {
			found = i;
		}
	}
	tree->children[found] = new_child;

	update_weights(tree, w);
}

template <class V>
MarkedNP<V> split_off_subtree(const NP<V>& tree, const NP<V>& subtree, NodeWeight<V> w) {
	// Cut off the subtree
	auto parent = subtree->parent.lock();
	subtree->parent = std::weak_ptr<Node<Number<K>>>();

	// Find sibling
	auto sibling = get_other_child(parent, subtree);

	// Contract
	auto grandparent = parent->parent.lock();
	sibling->parent = grandparent;

	if (grandparent != nullptr) {
		replace_child(grandparent, parent, sibling, w);
		return {tree, sibling};
	} else {
		return {sibling, sibling};
	}
}

template <class V>
std::pair<NP<V>, NP<V>> ancestor_search(const NP<V>& node, const Number<K>& max_weight, NodeWeight<V> w) {
	NPV mu_hat = node;
	NPV mu_star = node->parent.lock();

	while (w(mu_star) < max_weight) {
		mu_hat = mu_star;
		mu_star = mu_hat->parent.lock();
	}

	return {mu_hat, mu_star};
}

template <class V>
NP<V> largest_leaf(NP<V> node, NodeWeight<V> w) {
	if (node->is_leaf())
		return node;
	NP<V> largest;
	Number<K> max_weight = -1;
	for (const auto& child : node->children) {
		auto leaf = largest_leaf(child, w);
		if (leaf->value > max_weight) {
			max_weight = w(leaf);
			largest = leaf;
		}
	}
	return largest;
}

template <class V>
NP<V> copy_tree(const NP<V>& tree, std::unordered_map<NP<V>, NP<V>>& copy_to_tree, NodeWeight<V> w) {
	if (tree->is_leaf()) {
		auto new_node = std::make_shared<Node<Number<K>>>(w(tree));
		copy_to_tree[new_node] = tree;
		return new_node;
	} else {
		auto node = std::make_shared<Node<Number<K>>>(w(tree));
		for (const auto& child : tree->children) {
			auto copy = copy_tree(child, copy_to_tree);
			node->add_child(copy);
		}
		copy_to_tree[node] = tree;
		return node;
	}
}

template <class V>
std::optional<NP<V>> largest_child(NP<V> node, NodeWeight<V> w) {
	if (node->children.empty()) {
		return std::nullopt;
	} else {
		auto children_copy = node->children;
		std::sort(children_copy.begin(), children_copy.end(), [w](const NP<V>& c1, const NP<V>& c2) { return w(c1) < w(c2); });
		return children_copy.back();
	}
}

/// Use longest processing time algorithm to partition a vector of nodes into two vectors.
template <class V>
std::pair<std::pair<std::vector<NP<V>>, Number<K>>, std::pair<std::vector<NP<V>>, Number<K>>>
lpt_partition(std::vector<NP<V>> nodes, NodeWeight<V> w) {
	std::vector<NP<V>> l1;
	std::vector<NP<V>> l2;

	Number<K> w1 = 0;
	Number<K> w2 = 0;

	for (const auto& n : nodes) {
		if (w1 <= w2) {
			l1.push_back(n);
			w1 += w(n);
		} else {
			l2.push_back(n);
			w2 += w(n);
		}
	}

	return {{l1, w1}, {l2, w2}};
}

template <class V>
NPV create_binary_npv(Number<K> weight, std::vector<NP<V>> children, std::unordered_map<NPV, NP<V>>& copy_to_tree, NodeWeight<V> w, std::optional<NP<V>> original) {
	// Children with non-zero weight
	std::vector<NP<V>> nz_children;
	for (const auto& child : children) {
		if (w(child) > 0) {
			nz_children.push_back(child);
		}
	}

	if (nz_children.size() == 0) {
		auto new_node = std::make_shared<Node<Number<K>>>(weight);
		if (original.has_value()) {
			copy_to_tree[new_node] = *original;
		}
		return new_node;
	} else if (nz_children.size() == 1) {
		throw std::runtime_error("Nodes with one child are not supported.");
	} else if (nz_children.size() == 2) {
		auto node = std::make_shared<Node<Number<K>>>(weight);
		if (original.has_value()) {
			copy_to_tree[node] = *original;
		}
		for (const auto& child : nz_children) {
			auto copy = create_binary_npv<V>(w(child), child->children, copy_to_tree, w, child);
			node->add_child(copy);
		}
		return node;
	} else {
		// Put half the children left and half right.
		Number<K> leftWeight = 0.0;
		std::vector<NP<V>> leftChildren;
		for (int i = 0; i < (nz_children.size() + 1) / 2; i++) {
			leftWeight += w(nz_children[i]);
			leftChildren.push_back(nz_children[i]);
		}

		Number<K> rightWeight = 0.0;
		std::vector<NP<V>> rightChildren;
		for (int i = (nz_children.size() + 1) / 2; i < nz_children.size(); i++) {
			rightWeight += w(nz_children[i]);
			rightChildren.push_back(nz_children[i]);
		}

		auto leftChild = create_binary_npv<V>(leftWeight, leftChildren, copy_to_tree, w, std::nullopt);
		auto rightChild = rightChildren.size() == 1 ?
		  create_binary_npv<V>(rightWeight, rightChildren[0]->children, copy_to_tree, w, rightChildren[0]) :
		  create_binary_npv<V>(rightWeight, rightChildren, copy_to_tree, w, std::nullopt);

		auto node = std::make_shared<Node<Number<K>>>(weight);
		node->add_child(leftChild);
		node->add_child(rightChild);
		if (original.has_value()) {
			copy_to_tree[node] = *original;
		}

		return node;
	}
}

template <class V>
NPD create_binary_npv_balanced(Number<K> weight, std::vector<NP<V>> children, int depth, std::unordered_map<NPD, NP<V>>& copy_to_tree, NodeWeight<V> w, std::optional<NP<V>> original) {
	// Children with non-zero weight
	std::vector<NP<V>> nz_children;
	for (const auto& child : children) {
		if (w(child) > 0) {
			nz_children.push_back(child);
		}
	}

	if (nz_children.size() == 0) {
		auto new_node = std::make_shared<Node<DepthWeight>>(DepthWeight{depth, weight});
		if (original.has_value()) {
			copy_to_tree[new_node] = *original;
		}
		return new_node;
	} else if (nz_children.size() == 1) {
		throw std::runtime_error("Nodes with one child are not supported.");
	} else if (nz_children.size() == 2) {
		auto node = std::make_shared<Node<DepthWeight>>(DepthWeight{depth, weight});
		if (original.has_value()) {
			copy_to_tree[node] = *original;
		}
		for (const auto& child : nz_children) {
			auto copy = create_binary_npv_balanced<V>(w(child), child->children, depth + 1, copy_to_tree, w, child);
			node->add_child(copy);
		}
		return node;
	} else {
		std::sort(children.begin(), children.end(), [w](const NP<V>& c1, const NP<V>& c2) { return w(c1) < w(c2); });

		auto node = std::make_shared<Node<DepthWeight>>(DepthWeight{depth, weight});

		NPD left_child;
		NPD right_child;

		if (w(children[0]) >= weight / 2) {
			const auto& child = children[0];
			left_child = create_binary_npv_balanced<V>(w(child), child->children, depth + 1, copy_to_tree, w, child);
			std::vector<NP<V>> remaining;
			std::copy(++children.begin(), children.end(), std::back_inserter(remaining));
			std::optional<NP<V>> original_new = std::nullopt;
			right_child = create_binary_npv_balanced(weight - w(child), remaining, depth, copy_to_tree, w, original_new);
		} else {
			auto [r1, r2] = lpt_partition(children, w);
			auto [l1, w1] = r1;
			auto [l2, w2] = r2;

			if (l1.size() == 1) {
				auto child = l1[0];
				left_child = create_binary_npv_balanced<V>(w1, child->children, depth + 1, copy_to_tree, w, child);
			} else {
				left_child = create_binary_npv_balanced<V>(w1, l1, depth, copy_to_tree, w, std::nullopt);
			}
			if (l2.size() == 1) {
				auto child = l2[0];
				right_child = create_binary_npv_balanced<V>(w2, child->children, depth + 1, copy_to_tree, w, child);
			} else {
				right_child = create_binary_npv_balanced<V>(w2, l2, depth, copy_to_tree, w, std::nullopt);
			}
		}

		node->add_child(left_child);
		node->add_child(right_child);
		if (original.has_value()) {
			copy_to_tree[node] = *original;
		}

		return node;
	}
}

template <class V>
NPV convert_to_binary_npv(NP<V>& tree, std::unordered_map<NPV, NP<V>>& copy_to_tree, NodeWeight<V> w) {
	return create_binary_npv<V>(w(tree), tree->children, copy_to_tree, w, tree);
}

template <class V>
NPD convert_to_binary_npv_balanced(NP<V>& tree, std::unordered_map<NPD, NP<V>>& copy_to_tree, NodeWeight<V> w) {
	return create_binary_npv_balanced<V>(w(tree), tree->children, 0, copy_to_tree, w, tree);
}
}

#endif //CARTOCROW_TREEMAP_HELPERS_H
