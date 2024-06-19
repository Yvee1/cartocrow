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
