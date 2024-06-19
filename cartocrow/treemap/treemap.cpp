#include "treemap.h"
#include "arrangement_helpers.h"

namespace cartocrow::treemap {
Treemap::Treemap(NPV tree, std::shared_ptr<Arrangement<K>> arr, std::unordered_map<NPV, FaceH> leaf_to_face,
                 Rectangle<K> rect):
      m_tree(std::move(tree)), m_arrangement(std::move(arr)), m_leaf_to_face(std::move(leaf_to_face)), m_rectangle(std::move(rect)) {
	for (const auto& [leaf, face] : m_leaf_to_face) {
		m_face_to_leaf[face] = leaf;
	}
}

Polygon<K> Treemap::node_region(const NPV& node) const {
	if (node->is_leaf()) {
		return face_to_polygon(m_leaf_to_face.at(node));
	} else {
		auto faces = node_faces(node);
		return faces_to_polygon(faces);
	}
}

std::unordered_set<FaceH> Treemap::node_faces(const NPV& node) const {
	std::unordered_set<FaceH> faces;
	node_faces_helper(node, faces);
	return faces;
}

void Treemap::node_faces_helper(const NPV& node, std::unordered_set<FaceH>& faces) const {
	if (node->is_leaf()) {
		faces.insert(m_leaf_to_face.at(node));
	} else {
		node_faces_helper(node->children.at(0), faces);
		node_faces_helper(node->children.at(1), faces);
	}
}
}