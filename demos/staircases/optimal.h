#ifndef CARTOCROW_OPTIMAL_H
#define CARTOCROW_OPTIMAL_H
#include "staircase.h"

class OptimalStaircaseSimplification {
  public:
	OptimalStaircaseSimplification() = default;
	void compute(const Staircase& s);
	Staircase reconstruct(const Staircase& s, int target) const;

  private:
	typedef std::pair<std::optional<Number<K>>, std::optional<int>> Cell;

	void clear();
	void prepare(const Staircase& s);
	static std::vector<Number<K>> compute_cumulative_horizontal_area(const Staircase& s);
	static std::vector<Number<K>> compute_cumulative_vertical_area(const Staircase& s);
	Number<K> compute_shortcut_area(const Staircase& s, bool for_x, int index0, int index1);

	std::vector<Number<K>> c_area_woven;
	std::vector<std::vector<Cell>> table;
};

class OptimalPainting : public GeometryPainting {
  public:
	OptimalPainting(const std::shared_ptr<Staircase>& staircase, int& target);
	void paint(GeometryRenderer& renderer) const override;
	void reset();
	OptimalStaircaseSimplification m_opt_simp;

  private:
	const std::shared_ptr<Staircase> m_staircase;
	int& m_target;
};

#endif //CARTOCROW_OPTIMAL_H
