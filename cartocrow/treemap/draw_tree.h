#ifndef CARTOCROW_DRAW_TREE_H
#define CARTOCROW_DRAW_TREE_H

#include "tree.h"
#include <unordered_map>

// Tidier drawings of trees
// Edward M. Reingold and John S. Tilford

template <class V>
void draw_tree_helper(Node<V> tree, std::unordered_map<Node<V>, std::pair<int, int>> node_to_pos) {
	if (tree.is_leaf()) {
		node_to_pos.insert({0, 0});
	} else {
		auto left = tree->children[0];
	}
}

#endif //CARTOCROW_DRAW_TREE_H
