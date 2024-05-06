#ifndef CARTOCROW_OPTIMAL_H
#define CARTOCROW_OPTIMAL_H
#include "staircase.h"

class OptimalStaircaseSimplification {
  public:
	OptimalStaircaseSimplification() = default;
	Staircase compute(const Staircase& s, int target);

  private:
	typedef std::pair<std::optional<Number<K>>, std::optional<int>> Cell;

	void clear();
	void prepare(const Staircase& s);
	static std::vector<Number<K>> compute_cumulative_horizontal_area(const Staircase& s);
	static std::vector<Number<K>> compute_cumulative_vertical_area(const Staircase& s);
	Number<K> compute_shortcut_area(const Staircase& s, bool for_x, int index0, int index1);
	Staircase reconstruct(const Staircase& s, int target);

	std::vector<Number<K>> cum_woven;
	std::vector<std::vector<Cell>> table;
};

#endif //CARTOCROW_OPTIMAL_H
