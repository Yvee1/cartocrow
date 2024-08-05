#include "parse_csv_to_tree.h"

std::string getNextLine(std::istream& str) {
	std::string line;
	std::getline(str,line);
	return line;
}

std::vector<std::string> splitIntoTokens(const std::string& line, char delimiter) {
	std::vector<std::string>   result;
	std::stringstream          lineStream(line);
	std::string                cell;

	while(std::getline(lineStream, cell, delimiter))
	{
		result.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty())
	{
		// If there was a trailing comma then add an empty element.
		result.emplace_back("");
	}
	return result;
}

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
	return splitIntoTokens(getNextLine(str), ',');
}

NPN parse_csv_to_tree(const std::string& csv, bool simplify_name) {
	std::stringstream ss(csv);

	std::unordered_map<std::string, NPN> nodes;
	std::optional<NPN> root_node;

	while (ss) {
		auto parts = getNextLineAndSplitIntoTokens(ss);
		if (parts.size() < 3) break;
		std::string name = parts[0];
		if (simplify_name) {
			auto nameParts = splitIntoTokens(name, '/');
			name = nameParts[nameParts.size() - 1];
		}
		std::string parent = parts[1];
		if (simplify_name) {
			auto parentParts = splitIntoTokens(parent, '/');
			parent = parentParts[parentParts.size() - 1];
		}
		std::vector<Number<K>> weights;

		for (int i = 2; i < parts.size(); i++) {
			weights.push_back(stod(parts[i]));
		}

		NPN node = std::make_shared<Node<Named>>(Named{name, weights});
		if (parent != "root") {
			auto parent_node = nodes.at(parent);
			parent_node->add_child(node);
		} else {
			root_node = node;
		}
		nodes.insert({name, node});
	}

	return *root_node;
}

std::function<Number<K>(NPN node)> timestep_weights(int i) {
	return [i](const auto& n) { return n->value.weight[i]; };
}