#ifndef CARTOCROW_PARSE_CSV_TO_TREE_H
#define CARTOCROW_PARSE_CSV_TO_TREE_H

#include "cartocrow/treemap/tree.h"
#include "cartocrow/treemap/treemap_helpers.h"

using namespace cartocrow;
using namespace cartocrow::treemap;

struct Named {
	std::string name;
	std::vector<Number<K>> weight;
};

typedef std::shared_ptr<Node<Named>> NPN;

NPN parse_csv_to_tree(const std::string& csv, bool simplify_name = true);

extern std::function<Number<K>(NPN node)> timestep_weights(int i);
#endif //CARTOCROW_PARSE_CSV_TO_TREE_H
