#include "parse_tree.h"

#include <iostream>

std::vector<std::string> find_subtrees(const std::string& tree) {
	if (tree[1] != '(') {
		return { tree.substr(1, tree.size() - 2) };
	} else {
		std::vector<std::string> substrings;

		int depth = 1;
		std::string substring;

		for (int i = 1; i < tree.size() - 1; i++) {
			char c = tree[i];
			substring += c;
			if (c == '(') {
				depth += 1;
			}
			if (c == ')') {
				depth -= 1;
			}
			if (depth == 1) {
				substrings.push_back(substring);
				substring = "";
			}
		}
		return substrings;
	}
}

std::shared_ptr<Node<double>> parse_tree(const std::string& tree) {
	auto subtrees = find_subtrees(tree);
	if (subtrees.size() == 1) {
		double w = stod(tree.substr(1, tree.size() - 2));
		return std::make_shared<Node<double>>(w);
	} else {
		double total_weight = 0;
		std::vector<std::shared_ptr<Node<double>>> children;
		for (const auto& subtree : subtrees) {
			auto child = parse_tree(subtree);
			children.push_back(child);
			total_weight += child->value;
		}
		auto node = std::make_shared<Node<double>>(total_weight);
		for (const auto& child : children) {
			node->add_child(child);
		}
		return node;
	}
}
