#include "move.h"

Contraction::Contraction(std::shared_ptr<Staircase> staircase, MoveBox box): m_box(box), m_staircase(std::move(staircase))  {}

void Contraction::execute() {
	int i = m_box.index;
	auto& xs = m_staircase->m_xs;
	auto& ys = m_staircase->m_ys;

	bool below = i % 2 == 0;

	if (below) {
		auto x_it = std::next(xs.begin(), floor(i / 2));
		m_erased_x = *x_it;
		xs.erase(x_it);
		auto y_it = std::next(ys.begin(), floor((i + 1) / 2 + 1));
		m_erased_y = *y_it;
		ys.erase(y_it);
	} else {
		auto x_it = std::next(xs.begin(), floor((i / 2) + 1));
		m_erased_x = *x_it;
		xs.erase(x_it);
		auto y_it = std::next(ys.begin(), floor((i + 1) / 2));
		m_erased_y = *y_it;
		ys.erase(y_it);
	}
}

void Contraction::undo() {
	int i = m_box.index;
	auto& xs = m_staircase->m_xs;
	auto& ys = m_staircase->m_ys;

	bool below = i % 2 == 0;

	if (below) {
		xs.insert(std::next(xs.begin(), floor(i / 2)), m_box.rectangle.xmin());
		ys.insert(std::next(ys.begin(), floor((i + 1) / 2 + 1)), m_box.rectangle.ymax());
	} else {
		xs.insert(std::next(xs.begin(), floor((i / 2) + 1)), m_box.rectangle.xmax());
		ys.insert(std::next(ys.begin(), floor((i + 1) / 2)), m_box.rectangle.ymin());
	}
}

EdgeMove::EdgeMove(std::shared_ptr<Staircase> staircase, Edge edge, Number<K> start_pos, Number<K> new_pos) :
      m_staircase(std::move(staircase)), m_edge(edge), m_start_pos(start_pos), m_new_pos(new_pos) {}

void EdgeMove::execute() {
	if (m_edge.vertical) {
		m_staircase->m_xs[m_edge.index] = m_new_pos;
	} else {
		m_staircase->m_ys[m_edge.index] = m_new_pos;
	}
}

void EdgeMove::undo() {
	if (m_edge.vertical) {
		m_staircase->m_xs[m_edge.index] = m_start_pos;
	} else {
		m_staircase->m_ys[m_edge.index] = m_start_pos;
	}
}


std::unique_ptr<Command> move_or_contract(const std::shared_ptr<Staircase>& staircase, Edge m_edge, Number<K> start_pos, Number<K> new_pos) {
	auto& xs = staircase->m_xs;
	auto& ys = staircase->m_ys;
	if (m_edge.vertical) {
		if (m_edge.index > 0 && xs[m_edge.index - 1] == new_pos) {
			xs[m_edge.index - 1] = start_pos;
			auto box = staircase->move(2 * m_edge.index - 1);
			xs[m_edge.index - 1] = new_pos;
			return std::make_unique<Contraction>(staircase, box);
		} else if (m_edge.index + 1 < xs.size() && xs[m_edge.index + 1] == new_pos) {
			xs[m_edge.index + 1] = start_pos;
			auto box = staircase->move(2 * m_edge.index);
			xs[m_edge.index + 1] = new_pos;
			return std::make_unique<Contraction>(staircase, box);
		} else {
			return std::make_unique<EdgeMove>(staircase, m_edge, start_pos, new_pos);
		}
	} else {
		if (m_edge.index > 0 && ys[m_edge.index - 1] == new_pos) {
			ys[m_edge.index - 1] = start_pos;
			auto box = staircase->move(2 * m_edge.index - 2);
			ys[m_edge.index - 1] = new_pos;
			return std::make_unique<Contraction>(staircase, box);
		} else if (m_edge.index + 1 < ys.size() && ys[m_edge.index + 1] == new_pos) {
			ys[m_edge.index + 1] = start_pos;
			auto box = staircase->move(2 * m_edge.index - 1);
			ys[m_edge.index + 1] = new_pos;
			return std::make_unique<Contraction>(staircase, box);
		} else {
			return std::make_unique<EdgeMove>(staircase, m_edge, start_pos, new_pos);
		}
	}
}
