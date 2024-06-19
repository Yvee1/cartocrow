#ifndef CARTOCROW_PARSE_TREE_H
#define CARTOCROW_PARSE_TREE_H

#include <vector>
#include <string>
#include "tree.h"

std::vector<std::string> find_subtrees(const std::string& tree);

template <class FT>
std::shared_ptr<Node<FT>> parse_tree(const std::string& tree) {
	auto subtrees = find_subtrees(tree);
	if (subtrees.size() == 1) {
		FT w = stod(tree.substr(1, tree.size() - 2));
		return std::make_shared<Node<FT>>(w);
	} else {
		FT total_weight = 0;
		std::vector<std::shared_ptr<Node<FT>>> children;
		for (const auto& subtree : subtrees) {
			auto child = parse_tree<FT>(subtree);
			children.push_back(child);
			total_weight += child->value;
		}
		auto node = std::make_shared<Node<FT>>(total_weight);
		for (const auto& child : children) {
			node->add_child(child);
		}
		return node;
	}
}

#endif //CARTOCROW_PARSE_TREE_H
