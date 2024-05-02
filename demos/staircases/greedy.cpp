#include "greedy.h"

std::vector<MoveBox> min_cost_moves(const Staircase& simp, const Staircase& input, const GreedyCost& cost) {
	std::vector<MoveBox> moves = simp.moves();

	std::optional<Number<K>> min_cost;
	for (const auto& m : moves) {
		auto c = cost(simp, input, m);
		if (!min_cost.has_value() || c < *min_cost) {
			min_cost = c;
		}
	}

	if (!min_cost.has_value())
		return {};

	std::vector<MoveBox> minimum;
	for (const auto& m : moves) {
		if (cost(simp, input, m) == min_cost) {
			minimum.push_back(m);
		}
	}

	return minimum;
}

std::optional<std::unique_ptr<Contraction>> greedy_contraction(std::shared_ptr<Staircase>& simp,
                                                               const Staircase& input,
                                                               GreedyStrategy strategy,
                                                               const GreedyCost& cost,
                                                               std::mt19937& gen) {
	auto& xs = simp->m_xs;
	auto& ys = simp->m_ys;

	if (xs.size() <= 1) return std::nullopt;

	if (strategy != GreedyStrategy::RANDOM) {
		std::optional<Number<K>> min_cost;
		std::optional<MoveBox> min_move;
		bool min_priority = false;

		for (int i = 0; i < 2 * xs.size() - 2; i++) {
			bool priority;
			if (strategy == GreedyStrategy::FIRST) {
				priority = true;
			} else if (strategy == GreedyStrategy::TOP) {
				priority = i % 2 != 0;
			} else if (strategy == GreedyStrategy::BOTTOM) {
				priority = i % 2 == 0;
			}

			auto move = simp->move(i);
			Number<K> c = cost(*simp, input, move);
			if (!min_cost.has_value() || c < *min_cost || !min_priority && priority && c <= *min_cost) {
				min_cost = c;
				min_move = move;
				min_priority = priority;
			}
		}

		if (!min_move.has_value()) return std::nullopt;
		return std::make_unique<Contraction>(simp, *min_move);
	} else {
		auto minimum = min_cost_moves(*simp, input, cost);
		std::uniform_int_distribution<int> dist(0, minimum.size() - 1);
		auto move = minimum[dist(gen)];
		return std::make_unique<Contraction>(simp, move);
	}
}

GreedyCost greedy_area = [](const Staircase& s, const Staircase& input, const MoveBox& box) {
	return box.rectangle.area();
};

GreedyCost greedy_bracket = [](const Staircase& s, const Staircase& input, const MoveBox& box) {
	auto bs = brackets(input, s);

	Bracket l = bs[box.index];
	Bracket m = bs[box.index + 1];
	Bracket r = bs[box.index + 2];

	return l.complexity() + m.complexity() + r.complexity();
};
