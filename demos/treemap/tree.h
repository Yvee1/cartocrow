#ifndef CARTOCROW_TREE_H
#define CARTOCROW_TREE_H

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

template <class V>
class Node : public std::enable_shared_from_this<Node<V>> {
  public:
	explicit Node(V v) : value(v) {};
	void add_child(std::shared_ptr<Node<V>> subtree) {
		children.push_back(subtree);
		subtree->parent = this->shared_from_this();
	}

	bool is_leaf() { return children.empty(); }

	std::vector<std::shared_ptr<Node<V>>> children;
	std::weak_ptr<Node<V>> parent;
	V value;
};

#endif //CARTOCROW_TREE_H
