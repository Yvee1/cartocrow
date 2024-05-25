#include "optimal.h"

#include <utility>
#include "colors.h"

void OptimalStaircaseSimplification::compute(const Staircase& s) {
	clear();
	int n = s.num_of_segments();

	prepare(s);

	int corner_count = n / 2;

	for (int i = 0; i < n; i++) {
		std::vector<Cell> row(corner_count, std::pair(std::nullopt, std::nullopt));
		table.push_back(row);
	}

	table[0][0] = std::pair(0, std::nullopt);

	for (int row = 1; row < n; row++) {
		bool other_dimension = row % 2;
		int current_dimension_index = floor(row / 2);

		for (int next_coord_index = current_dimension_index; next_coord_index < corner_count; next_coord_index++) {
			Cell min_val(std::nullopt, std::nullopt);
			for (int p_coord = 0; p_coord < next_coord_index + other_dimension; p_coord++) {
				auto area = table[row-1][p_coord].first;
				if (!area.has_value()) continue;
				auto added_area = compute_shortcut_area(s, other_dimension, p_coord, next_coord_index);
				auto new_area = *area + added_area;
				if (!min_val.first.has_value() || new_area < *min_val.first) {
					min_val = std::pair(new_area, p_coord);
				}
			}
			table[row][next_coord_index] = min_val;
		}
	}
}

void OptimalStaircaseSimplification::prepare(const Staircase& s) {
	auto h_c_area = compute_cumulative_horizontal_area(s);
	auto v_c_area = compute_cumulative_vertical_area(s);

	c_area_woven = {};
	for (int i = 0; i < v_c_area.size() * 2; i++) {
		if (i % 2 == 0) {
			c_area_woven.push_back(v_c_area[i / 2]);
		} else {
			c_area_woven.push_back(h_c_area[i / 2]);
		}
	}
}

std::vector<Number<K>> OptimalStaircaseSimplification::compute_cumulative_horizontal_area(const Staircase& s) {
	std::vector<Number<K>> h_c_area = {0};

	for (int i = 1; i < s.m_xs.size(); i++) {
		auto x_diff = s.m_xs[i] - s.m_xs[i-1];
		auto y = s.m_ys[i];
		h_c_area.push_back(h_c_area.back() + x_diff * y);
	}

	return h_c_area;
}

std::vector<Number<K>> OptimalStaircaseSimplification::compute_cumulative_vertical_area(const Staircase& s) {
	std::vector<Number<K>> v_c_area = {0, 0};

	for (int i = 2; i < s.m_ys.size(); i++) {
		auto y_diff = s.m_ys[i] - s.m_ys[i-1];
		auto x = s.m_xs[i-1];
		v_c_area.push_back(v_c_area.back() + y_diff * x);
	}

	return v_c_area;
}

Number<K> OptimalStaircaseSimplification::compute_shortcut_area(const Staircase& s, bool for_x, int i, int j) {
	if (for_x && j - i == 0 || !for_x && j - i == 1) return 0;
	int sc_i = 2 * i + 1 - for_x;
	int sc_j = 2 * j + for_x;

	auto added_area = c_area_woven[sc_j] - c_area_woven[sc_i+1];
	added_area -= (s.coord(sc_j) - s.coord(sc_i + 1)) * s.coord(sc_i);
	return added_area;
}

void OptimalStaircaseSimplification::clear() {
	table.clear();
	c_area_woven.clear();
}

Staircase OptimalStaircaseSimplification::reconstruct(const Staircase& s, int target) const {
	auto element = table[target-1].back();
	std::vector<Number<K>> xs = { s.m_xs.back() };
	std::vector<Number<K>> ys;
	for (int i = 0; i < target-1; i++) {
		auto parent = element.second;
		if (i % 2 == 0) {
			ys.push_back(s.m_ys.at(*parent));
		} else {
			xs.push_back(s.m_xs.at(*parent));
		}
		element = table[target - 1 - (i+1)][*parent];
	}
	std::reverse(xs.begin(), xs.end());
	std::reverse(ys.begin(), ys.end());

	return Staircase(xs, ys);
}

OptimalPainting::OptimalPainting(const std::shared_ptr<Staircase>& staircase, int& target)
    : m_staircase(staircase), m_target(target) {
	m_opt_simp.compute(*m_staircase);
}

void OptimalPainting::paint(cartocrow::renderer::GeometryRenderer& renderer) const {
	auto s = m_opt_simp.reconstruct(*m_staircase, m_target);
	renderer.setStroke(CB::green, 2.5);
	draw_staircase(renderer, s);
}

void OptimalPainting::reset() {
	m_opt_simp.compute(*m_staircase);
}
