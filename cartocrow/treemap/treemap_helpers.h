//
// Created by steven on 6/1/24.
//

#ifndef CARTOCROW_TREEMAP_HELPERS_H
#define CARTOCROW_TREEMAP_HELPERS_H

#include "tree.h"
#include "rectangle_helpers.h"
#include "cartocrow/core/core.h"
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

namespace cartocrow::treemap {
//typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt K;
typedef Inexact K;

//struct Value {
//	int label;
//	Number<K> weight;
//};

template <class V> using NP = std::shared_ptr<Node<V>>;
typedef NP<Number<K>> NPV;

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
}

#endif //CARTOCROW_TREEMAP_HELPERS_H
