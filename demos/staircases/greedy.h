#ifndef CARTOCROW_GREEDY_H
#define CARTOCROW_GREEDY_H

#include "move.h"
#include "brackets.h"
#include <random>

enum GreedyStrategy {
	FIRST,
	RANDOM,
	TOP,
	BOTTOM,
};

typedef std::function<Number<K>(const Staircase&, const Staircase&, const MoveBox&)> GreedyCost;

std::vector<MoveBox> min_cost_moves(const Staircase& simp, const Staircase& input, const GreedyCost& cost);

std::optional<std::unique_ptr<Contraction>> greedy_contraction(std::shared_ptr<Staircase>& simp,
                                                               const Staircase& input,
                                                               GreedyStrategy strategy,
                                                               const GreedyCost& cost,
                                                               std::mt19937& gen);

extern GreedyCost greedy_area;
extern GreedyCost greedy_bracket;

#endif //CARTOCROW_GREEDY_H

