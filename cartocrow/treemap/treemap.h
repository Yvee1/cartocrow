#ifndef CARTOCROW_TREEMAP_H
#define CARTOCROW_TREEMAP_H

#include "treemap_helpers.h"
#include "arrangement_helpers.h"

namespace cartocrow::treemap {
template <class V>
class Treemap {
  private:
	void node_faces_helper(const NP<V>& node, std::unordered_set<FaceH>& faces) const {
		if (node->is_leaf()) {
			if (m_leaf_to_face.contains(node)) { // if node has weight zero it is not part of the treemap
				faces.insert(m_leaf_to_face.at(node));
			}
		} else {
			for (const auto& child : node->children) {
				node_faces_helper(child, faces);
			}
		}
	}

  public:
	Treemap(NP<V> tree, std::shared_ptr<Arrangement<K>> arr, std::unordered_map<NP<V>, FaceH> leaf_to_face, Rectangle<K> rect):
	      m_tree(std::move(tree)), m_arrangement(std::move(arr)), m_leaf_to_face(std::move(leaf_to_face)), m_rectangle(std::move(rect)) {
		for (const auto& [leaf, face] : m_leaf_to_face) {
			m_face_to_leaf[face] = leaf;
		}
	};
	std::unordered_set<FaceH> node_faces(const NP<V>& node) const {
		std::unordered_set<FaceH> faces;
		node_faces_helper(node, faces);
		return faces;
	};
	std::optional<Polygon<K>> node_region(const NP<V>& node) const {
		if (node->is_leaf()) {
			if (m_leaf_to_face.contains(node)) {
				return face_to_polygon(m_leaf_to_face.at(node));
			} else {
				return std::nullopt; // This should happen only if the node has weight zero
			}
		} else {
			auto faces = node_faces(node);
			return faces_to_polygon(faces);
		}
	};
	std::shared_ptr<Arrangement<K>> m_arrangement;
	std::unordered_map<NP<V>, FaceH> m_leaf_to_face;
	std::unordered_map<FaceH, NP<V>> m_face_to_leaf;
	NP<V> m_tree;
	Rectangle<K> m_rectangle;
};
}

#endif //CARTOCROW_TREEMAP_H
