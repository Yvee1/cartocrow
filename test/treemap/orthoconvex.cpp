#include "../catch.hpp"

#include "cartocrow/treemap/orthoconvex.h"
#include "cartocrow/treemap/parse_tree.h"
#include "cartocrow/treemap/aspect_ratio.h"

using namespace cartocrow;
using namespace cartocrow::treemap;

void check_weights(const Treemap<Number<K>>& treemap, const NPV& tree) {
	auto poly = treemap.node_region(tree);
	if (!poly.has_value()) {
		CHECK(tree->value == 0);
		return;
	}
	CHECK(abs(poly->area()) / abs(treemap.m_rectangle.area()) == tree->value / treemap.m_tree->value);
	for (const auto& child : tree->children) {
		check_weights(treemap, child);
	}
}

void check_aspect_ratio(const Treemap<Number<K>>& treemap, const NPV& tree) {
	auto poly = treemap.node_region(tree);
	if (!poly.has_value()) {
		CHECK(tree->value == 0);
		return;
	}
	auto ar = aspect_ratio_square_percentage(*poly);
	if (tree->is_leaf()) {
		if (poly->size() == 4) {
			CHECK(ar <= 8);
		} else {
			CHECK(ar <= 32);
		}
	} else {
		CHECK(ar <= 64);
	}

	for (const auto& child : tree->children) {
		check_aspect_ratio(treemap, child);
	}
}

TEST_CASE("Orthoconvex case (a).") {
	for (const auto input : {"((7)(1))", "((1)(10000))", "((10000)(1))", "((5)(5))", "(((100)(1))((30)(30)))"}) {
		for (int i = 0; i < 4; i++) {
			auto corner = static_cast<Corner>(i);
			auto root = parse_tree<Number<K>>(input);
			auto treemap = build_treemap(root, corner);

			SECTION("Weights.") {
				check_weights(treemap, root);
			}

			SECTION("Aspect ratio.") {
				check_aspect_ratio(treemap, root);
			}
		}
	}
}

TEST_CASE("Orthoconvex case (b).") {
	auto inputs = {
	    "(((4)((100)(1)))((4)(5)))",
	};
	for (const auto input : inputs) {
		for (int i = 0; i < 4; i++) {
			auto corner = static_cast<Corner>(i);
			auto root = parse_tree<Number<K>>(input);
			auto treemap = build_treemap(root);

			SECTION("Weights.") {
				check_weights(treemap, root);
			}

			SECTION("Aspect ratio.") {
				check_aspect_ratio(treemap, root);
			}
		}
	}
}

TEST_CASE("Orthoconvex case (c).") {
	auto inputs = {
	    // # T' is non-empty
	    // Lambda is L-shape
	    "(((100)(1))((4)(5)))",
		// Lambda is S-shape
	    "(((100)(1))((8.5)(0.5)))",
		// # T' is empty
	    "(((100)(1))(9))"
	};
	for (const auto input : inputs) {
		for (int i = 0; i < 4; i++) {
			auto corner = static_cast<Corner>(i);
			auto root = parse_tree<Number<K>>(input);
			auto treemap = build_treemap(root);

			SECTION("Weights.") {
				check_weights(treemap, root);
			}

			SECTION("Aspect ratio.") {
				check_aspect_ratio(treemap, root);
			}
		}
	}
}

TEST_CASE("Orthoconvex case (d).") {
	auto inputs = {
	    "(((100)(1))((((1)(1))((1)(1)))(((1)(1))(((1)(1))(1)))))"
	};
	for (const auto input : inputs) {
		SECTION("Input: " + std::string(input)) {
			for (int i = 0; i < 4; i++) {
				auto corner = static_cast<Corner>(i);
				SECTION("Corner: " + std::to_string(corner)) {
					auto root = parse_tree<Number<K>>(input);
					auto treemap = build_treemap(root);

					SECTION("Weights.") {
						check_weights(treemap, root);
					}

					SECTION("Aspect ratio.") {
						check_aspect_ratio(treemap, root);
					}
				}
			}
		}
	}
}