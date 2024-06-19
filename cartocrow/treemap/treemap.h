#ifndef CARTOCROW_TREEMAP_H
#define CARTOCROW_TREEMAP_H

#include "treemap_helpers.h"

namespace cartocrow::treemap {
class Treemap {
  public:
	Treemap(NPV tree, std::shared_ptr<Arrangement<K>> arr, std::unordered_map<NPV, FaceH> leaf_to_face, Rectangle<K> rect);
	Polygon<K> node_region(const NPV& node) const;
	std::unordered_set<FaceH> node_faces(const NPV& node) const;
	std::shared_ptr<Arrangement<K>> m_arrangement;
	std::unordered_map<NPV, FaceH> m_leaf_to_face;
	std::unordered_map<FaceH, NPV> m_face_to_leaf;
	NPV m_tree;
	Rectangle<K> m_rectangle;

  private:
	void node_faces_helper(const NPV& node, std::unordered_set<FaceH>& faces) const;
};
}

#endif //CARTOCROW_TREEMAP_H
