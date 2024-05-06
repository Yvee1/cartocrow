#include "optimal.h"

Staircase OptimalStaircaseSimplification::compute(const Staircase& s, int target) {
	clear();
	int n = s.num_of_segments();
	if (target % 2 != 0 || target > n || target < 0) {
		throw std::runtime_error("The target complexity should be a non-negative odd number smaller than the input complexity.");
	}

	prepare(s);

	int corner_count = n / 2;

	for (int i = 0; i < n; i++) {
		std::vector<Cell> row(corner_count, std::pair(std::nullopt, std::nullopt));
		table.push_back(row);
	}

	table[0][0] = std::pair(0, std::nullopt);

	for (int row = 1; row < n; row++) {
		std::cout << "row " << row << std::endl;
		bool other_dimension = row % 2;
		int current_dimension_index = floor(row / 2);

		for (int next_coord_index = current_dimension_index; next_coord_index < corner_count; next_coord_index++) {
			std::cout << "\tnext coord index " << next_coord_index << std::endl;
			Cell min_val(std::nullopt, std::nullopt);
			for (int p_coord = 0; p_coord < next_coord_index + other_dimension; p_coord++) {
				std::cout << "\t\tp_coord " << p_coord << std::endl;
				auto area = table[row-1][p_coord].first;
				if (!area.has_value()) continue;
				auto added_area = compute_shortcut_area(s, other_dimension, p_coord, next_coord_index);
				auto new_area = *area + added_area;
				std::cout << "\t\tnew_area  " << new_area << std::endl;
				if (!min_val.first.has_value() || new_area < *min_val.first) {
					min_val = std::pair(new_area, p_coord);
				}
			}
			table[row][next_coord_index] = min_val;
		}
	}

	std::cout << "AOSD: " << *(table[target - 1].back().first) << std::endl;
	return reconstruct(s, target);
}

void OptimalStaircaseSimplification::prepare(const Staircase& s) {
	auto h_cum = compute_cumulative_horizontal_area(s);
	auto v_cum = compute_cumulative_vertical_area(s);

	cum_woven = {};
	for (int i = 0; i < v_cum.size() * 2; i++) {
		if (i % 2 == 0) {
			cum_woven.push_back(v_cum[i / 2]);
		} else {
			cum_woven.push_back(h_cum[i / 2]);
		}
	}
	std::cout << h_cum.size() << std::endl;
	std::cout << v_cum.size() << std::endl;
	std::cout << s.num_of_segments() / 2 << std::endl;

	std::cout << "Cumulative horizontal area" << std::endl;
	for (auto h : h_cum) {
		std::cout << h << std::endl;
	}

	std::cout << "Cumulative vertical area" << std::endl;
	for (auto v : v_cum) {
		std::cout << v << std::endl;
	}
}

std::vector<Number<K>> OptimalStaircaseSimplification::compute_cumulative_horizontal_area(const Staircase& s) {
	std::vector<Number<K>> h_cum = {0};

	for (int i = 1; i < s.m_xs.size(); i++) {
		auto x_diff = s.m_xs[i] - s.m_xs[i-1];
		auto y = s.m_ys[i];
		h_cum.push_back(h_cum.back() + x_diff * y);
	}

	return h_cum;
}

std::vector<Number<K>> OptimalStaircaseSimplification::compute_cumulative_vertical_area(const Staircase& s) {
	std::vector<Number<K>> v_cum = {0, 0};

	for (int i = 2; i < s.m_ys.size(); i++) {
		auto y_diff = s.m_ys[i] - s.m_ys[i-1];
		auto x = s.m_xs[i-1];
		v_cum.push_back(v_cum.back() + y_diff * x);
	}

	return v_cum;
}

Number<K> OptimalStaircaseSimplification::compute_shortcut_area(const Staircase& s, bool for_x, int i, int j) {
	if (for_x && j - i == 0 || !for_x && j - 1 == 1) return 0;
	int sc_i = 2 * i + 1 - for_x;
	int sc_j = 2 * j + for_x;

	auto added_area = cum_woven[sc_j] - cum_woven[sc_i+1];
	added_area -= (s.coord(sc_j) - s.coord(sc_i + 1)) * s.coord(sc_i);
	return added_area;
}

void OptimalStaircaseSimplification::clear() {
	table.clear();
}

Staircase OptimalStaircaseSimplification::reconstruct(const Staircase& s, int target) {
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