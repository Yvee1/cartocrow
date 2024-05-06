#include "brackets.h"

bool supported_by(Segment<K> segment, Segment<K> other) {
	return segment.has_on(other.source()) && segment.has_on(other.target());
}

std::vector<Bracket> brackets(const Staircase& input, const Staircase& simplification) {
	std::vector<Bracket> bs;

	auto sim_steps = simplification.edges();
	auto inp_steps = input.edges();

	int j = -1;

	for (const auto& step : sim_steps) {
		int prev_j = j;

		while (j < 0 || j < inp_steps.size() && !supported_by(step.segment, inp_steps[j].segment)) {
			++j;
		}

		bs.emplace_back(prev_j, j);
	}
	bs.emplace_back(j, inp_steps.size());

	return bs;
}


BracketPainting::BracketPainting(const std::shared_ptr<Staircase>& input,
                                 const std::shared_ptr<Staircase>& simplification,
                                 QCheckBox* show_bracket_complexity,
                                 QCheckBox* show_bracket_dimensions):
      m_input(input),
      m_simplification(simplification),
      m_show_bracket_complexity(show_bracket_complexity),
      m_show_bracket_dimensions(show_bracket_dimensions) {}

void BracketPainting::paint(GeometryRenderer& renderer) const {
	if (!m_simplification->supported_by(*m_input)) return;
	auto bs = brackets(*m_input, *m_simplification);

	for (const auto& b : bs) {
		auto maybe_poly = b.polygon(*m_input);
		if (maybe_poly.has_value()) {
			renderer.setMode(GeometryRenderer::fill);
			renderer.setFill(Color(255, 200, 200));
			renderer.draw(*maybe_poly);
			renderer.setMode(GeometryRenderer::stroke);
			renderer.setStroke(Color(255, 0, 0), 1);
			if (m_show_bracket_complexity->isChecked()) {
				Point<K> text_pos;
				auto bb = maybe_poly->bbox();
				if (b.start % 2 == 0) {
					text_pos = Point<K>(bb.xmin(), bb.ymax()) + Vector<K>(0.5, -0.5);
				} else {
					text_pos = Point<K>(bb.xmax(), bb.ymin()) + Vector<K>(-0.5, 0.5);
				}
				renderer.drawText(text_pos, std::to_string(b.complexity()));
			}
			if (m_show_bracket_dimensions->isChecked()) {

			}
		}
	}
}