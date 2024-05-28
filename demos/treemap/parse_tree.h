#ifndef CARTOCROW_PARSE_TREE_H
#define CARTOCROW_PARSE_TREE_H

#include <vector>
#include <string>
#include "tree.h"

std::shared_ptr<Node<double>> parse_tree(const std::string& tree);

#endif //CARTOCROW_PARSE_TREE_H
