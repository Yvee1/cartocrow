#ifndef CARTOCROW_TREEMAP_HELPERS_H
#define CARTOCROW_TREEMAP_HELPERS_H

#include "tree.h"
#include "rectangle_helpers.h"
#include "cartocrow/core/core.h"
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

namespace cartocrow::treemap {
typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt K;
//typedef Inexact K;

//struct Value {
//	int label;
//	Number<K> weight;
//};

template <class V> using NP = std::shared_ptr<Node<V>>;
template <class V> using CNP = std::shared_ptr<const Node<V>>;
typedef NP<Number<K>> NPV;
typedef CNP<Number<K>> CNPV;

struct MarkedNPV {
	NPV tree;
	NPV marked;
};

//std::pair<NPV, int> label_tree(const NP<Number<K>>& tree, int start = 0);

typedef Arrangement<K>::Face_handle FaceH;

Polygon<K> face_to_polygon(const Arrangement<K>::Face_const_handle& face);
Rectangle<K> face_to_rectangle(const Arrangement<K>::Face_const_handle& face);
NPV get_other_child(const NPV& tree, const NPV& child);
void update_weights(NPV& tree);
void replace_child(NPV& tree, const NPV& old_child, const NPV& new_child);
MarkedNPV split_off_subtree(const NPV& tree, const NPV& subtree);
std::pair<FaceH, FaceH> slice_rectangle(Arrangement<K>& arr, FaceH& face,
										const Number<K>& corner_ratio, Corner corner,
										std::optional<bool> force_split_dir = std::nullopt);
std::pair<FaceH, FaceH> create_notch(Arrangement<K>& arr, const Rectangle<K>& rect,
									 const Number<K>& notch_ratio, Corner corner);
std::pair<FaceH, FaceH> slice_L_rectangle(Arrangement<K>& arr, FaceH& face,
										  const Number<K>& corner_ratio, Corner corner);
std::pair<FaceH, std::pair<FaceH, FaceH>>
slice_S_rectangle(Arrangement<K>& arr, FaceH& face, const Number<K>& corner_ratio,
				  const Number<K>& opposite_ratio, Corner corner);
std::pair<NPV, NPV> ancestor_search(const NPV& node, const Number<K>& max_weight);
NPV range_search(const NPV& tree, const Number<K>& max);
NPV largest_leaf(NPV node);
NPV copy_tree(const NPV& tree, std::unordered_map<NPV, NPV>& map);


template <class V>
NPV create_binary_npv(Number<K> weight, std::vector<NP<V>> children, std::unordered_map<NPV, NP<V>>& copy_to_tree, std::function<Number<K>(NP<V>)> w, std::optional<NP<V>> original) {
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

//		auto leftChild = std::make_shared<Node<Number<K>>>(leftWeight);
		auto leftChild = create_binary_npv<V>(leftWeight, leftChildren, copy_to_tree, w, std::nullopt);
		auto rightChild = rightChildren.size() == 1 ?
		  create_binary_npv<V>(rightWeight, rightChildren[0]->children, copy_to_tree, w, rightChildren[0]) :
		  create_binary_npv<V>(rightWeight, rightChildren, copy_to_tree, w, std::nullopt);

//		auto rightChild = std::make_shared<Node<Number<K>>>(rightWeight);
//		for (int i = children.size() / 2; i < children.size(); i++) {
//			auto child_copy = create_binary_npv(children[i], copy_to_tree, w);
//			rightChild->add_child(child_copy);
//		}
		auto node = std::make_shared<Node<Number<K>>>(leftWeight + rightWeight);
		node->add_child(leftChild);
		node->add_child(rightChild);
		if (original.has_value()) {
			copy_to_tree[node] = *original;
		}

		return node;
	}
}

template <class V>
NPV convert_to_binary_npv(NP<V>& tree, std::unordered_map<NPV, NP<V>>& copy_to_tree, std::function<Number<K>(NP<V>)> w) {
	return create_binary_npv<V>(w(tree), tree->children, copy_to_tree, w, tree);
}
}

#endif //CARTOCROW_TREEMAP_HELPERS_H
