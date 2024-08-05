#include "orthoconvex.h"

namespace cartocrow::treemap {
Treemap<Number<K>> build_treemap(NP<Number<K>>& tree, Corner corner) {
	return build_treemap<Number<K>>(tree, [](const auto& n) { return n->value; }, corner);
}

void recurse_treemap(const NPV& tree, const NPV& marked, Arrangement<K>& arr, FaceH& face,
                     Corner corner, std::unordered_map<NPV, FaceH>& leaf_regions) {
	if (tree->is_leaf()) {
		leaf_regions[tree] = face;
		return;
	}
	auto weight = tree->value;

	if (marked->value >= weight / 8) {
		// Case (a): marked is non-tiny: cut off small or large subtree, or huge leaf.
		auto nu = range_search(marked, 7 * weight / 8);
		auto ratio = nu->value / tree->value;

		FaceH cornerH;
		FaceH oppositeH;
		if (ratio <= 0.875) {
			std::tie(cornerH, oppositeH) = slice_rectangle(arr, face, ratio, corner);
		} else {
			std::tie(cornerH, oppositeH) = slice_L_rectangle(arr, face, ratio, corner);
		}
		auto remainder = split_off_subtree(tree, nu);
		recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner, leaf_regions);
		recurse_treemap(nu, nu, arr, cornerH, corner, leaf_regions);
		return;
	} else {
		auto [mu_hat, mu_star] = ancestor_search(marked, 7 * weight / 8);
		if (mu_hat->value >= weight / 8) {
			// Case (b): mu_hat is small or large: cut off mu_hat
			auto ratio = mu_hat->value / tree->value;
			auto [cornerH, oppositeH] = slice_rectangle(arr, face, ratio, corner);
			auto remainder = split_off_subtree(tree, mu_hat);
			recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner, leaf_regions);
			recurse_treemap(mu_hat, marked, arr, cornerH, corner, leaf_regions);
			return;
		}
		auto lambda = largest_leaf(get_other_child(mu_star, mu_hat));

		if (lambda->value >= weight / 4) {
			// Case (c): tree has large or huge leaf
			auto tree_minus_lambda = split_off_subtree(tree, lambda);
			if (tree_minus_lambda.tree == mu_hat) {
				auto ratio = lambda->value / weight;
				auto [lH, rectH] = slice_L_rectangle(arr, face, ratio, opposite(corner));
				recurse_treemap(lambda, lambda, arr, lH, corner, leaf_regions);
				recurse_treemap(tree_minus_lambda.tree, marked, arr, rectH, corner, leaf_regions);
				return;
			} else {
				auto remainder = split_off_subtree(tree_minus_lambda.tree, mu_hat);
				if (remainder.tree->value <= weight / 8) {
					auto [sH, rectsH] =
					    slice_S_rectangle(arr, face, mu_hat->value / weight,
					                      remainder.tree->value / weight, corner);
					auto [cornerH, oppositeH] = rectsH;
					recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner,
					                leaf_regions);
					recurse_treemap(lambda, lambda, arr, sH, corner, leaf_regions);
					recurse_treemap(mu_hat, marked, arr, cornerH, corner, leaf_regions);
					return;
				} else {
					auto [cornerH, oppositeH] = slice_rectangle(
					    arr, face, 1 - remainder.tree->value / weight, corner, true);
					auto [lH, rectH] = slice_L_rectangle(arr, cornerH, lambda->value / (lambda->value + mu_hat->value),
					                                     opposite(corner));
					recurse_treemap(remainder.tree, remainder.marked, arr, oppositeH, corner,
					                leaf_regions);
					recurse_treemap(lambda, lambda, arr, lH, corner, leaf_regions);
					recurse_treemap(mu_hat, marked, arr, rectH, corner, leaf_regions);
					return;
				}
			}
		}

		// Case (d)
		auto nu_hat = range_search(get_other_child(mu_star, mu_hat), 6 * weight / 8);
		auto nu = range_search(get_other_child(mu_star, mu_hat), weight / 4);
		auto tree_prime_1 = split_off_subtree(tree, mu_hat);
		auto tree_prime = split_off_subtree(tree_prime_1.tree, nu_hat);

		auto nu_hat_prime = split_off_subtree(nu_hat, nu);
		NPV remainder =
		    std::make_shared<Node<Number<K>>>(mu_hat->value + nu->value);
		remainder->add_child(nu);
		remainder->add_child(mu_hat);

		auto rect = face_to_rectangle(face);
		auto vertical = width(rect) >= height(rect);
		auto [cornerH, oppositeH] =
		    slice_rectangle(arr, face, remainder->value / weight, corner, vertical);
		auto [corner2H, opposite2H] =
		    slice_rectangle(arr, oppositeH, nu_hat_prime.tree->value / (weight - remainder->value), corner, vertical);
		recurse_treemap(remainder, marked, arr, cornerH, corner, leaf_regions);
		Corner flipped_corner = mirror_corner(corner, vertical);
		recurse_treemap(nu_hat_prime.tree, nu_hat_prime.marked, arr, corner2H, flipped_corner,
		                leaf_regions);
		recurse_treemap(tree_prime.tree, tree_prime.marked, arr, opposite2H, corner, leaf_regions);
		return;
	}
}
}