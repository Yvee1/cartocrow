/*
The CartoCrow library implements algorithmic geo-visualization methods,
developed at TU Eindhoven.
Copyright (C) 2021  Netherlands eScience Center and TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3f of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "staircases_demo.h"
#include <QApplication>
#include <QDockWidget>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <utility>

StaircaseDemo::StaircaseDemo() {
	// =======================================
	// ============  STAIRCASES ==============
	// =======================================

	//	auto input = std::make_shared<UniformStaircase>(20);
	//	auto input = std::make_shared<Staircase>(std::vector<double>({0, 1, 5, 6, 10}), std::vector<double>({0, 3, 4, 6, 8, 10}));
	auto input = std::make_shared<Staircase>(
	    std::vector<Number<K>>({0, 1, 5, 6, 10, 11, 20, 21, 26, 30, 33, 35, 37}),
	    std::vector<Number<K>>({0, 3, 4, 6, 8,  10, 12, 14, 19, 25, 26, 27, 29, 30})
	);
	auto s = std::make_shared<Staircase>(*input);

	// =======================================
	// ========= GEOMETRY WIDGET SETUP =======
	// =======================================
	setWindowTitle("Staircase simplification");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

	m_renderer->m_editables.push_back(std::make_unique<StaircaseEditable>(m_renderer, s, input));

	// =======================================
	// =============== WIDGETS ===============
	// =======================================
	auto* dockWidget = new QDockWidget();
	addDockWidget(Qt::RightDockWidgetArea, dockWidget);
	auto* vWidget = new QWidget();
	auto* vLayout = new QVBoxLayout(vWidget);
	vLayout->setAlignment(Qt::AlignTop);
	dockWidget->setWidget(vWidget);

	auto* constructionSection = new QLabel("<h3>Construction</h3>");
	vLayout->addWidget(constructionSection);

	auto* uniformN = new QSpinBox();
	uniformN->setValue(10);
	uniformN->setMinimum(1);
	uniformN->setMaximum(1000);
	auto* uniformNLabel = new QLabel("Uniform n");
	uniformNLabel->setBuddy(uniformN);
	vLayout->addWidget(uniformNLabel);
	vLayout->addWidget(uniformN);

	auto* create_uniform = new QPushButton("Create uniform staircase");
	vLayout->addWidget(create_uniform);

	auto* reset_button = new QPushButton("Reset");
	vLayout->addWidget(reset_button);

	auto* infoSection = new QLabel("<h3>Info</h3>");
	vLayout->addWidget(infoSection);

	auto* show_bracket_complexity = new QCheckBox("Show bracket complexity");
	vLayout->addWidget(show_bracket_complexity);
	show_bracket_complexity->setCheckState(Qt::CheckState::Checked);

	auto* show_bracket_dimensions = new QCheckBox("Show bracket dimension");
//	vLayout->addWidget(show_bracket_dimensions);

	auto* l_label = new QLabel("Distributed complexity: ");
	vLayout->addWidget(l_label);

	auto* greedySection = new QLabel("<h3>Greedy</h3>");
	vLayout->addWidget(greedySection);

	auto* greedy_step = new QPushButton("Step");
	vLayout->addWidget(greedy_step);

	// =========== EVENT HANDLERS ============
	connect(reset_button, &QPushButton::clicked, [s, input, this] {
		s->m_xs = input->m_xs;
		s->m_ys = input->m_ys;
		m_renderer->repaint();
	});
	connect(create_uniform, &QPushButton::clicked, [uniformN, s, input, this] {
		auto uni = UniformStaircase(uniformN->value());
		input->m_xs = uni.m_xs;
	  	input->m_ys = uni.m_ys;
	    s->m_xs = input->m_xs;
	    s->m_ys = input->m_ys;
	  	m_renderer->repaint();
	});
	connect(show_bracket_complexity, &QCheckBox::stateChanged, [this]{ m_renderer->repaint(); });
//	connect(show_bracket_dimensions, &QCheckBox::stateChanged, [this]{ m_renderer->repaint(); });
	connect(greedy_step, &QPushButton::clicked, [s, this] {
		do_greedy_step(*s);
		m_renderer->repaint();
//		m_update();
	});

	// =======================================
	// ============== PAINTINGS ==============
	// =======================================
	auto gridP = std::make_shared<GridPainting>(input);
	auto bracketP = std::make_shared<BracketPainting>(input, s, show_bracket_complexity, show_bracket_dimensions);
//	auto movesP = std::make_shared<MovesPainting>(s);
	auto inputP = std::make_shared<StaircasePainting>(input, true);
	auto sP = std::make_shared<StaircasePainting>(s, false);
	m_renderer->addPainting(gridP, "Grid");
	m_renderer->addPainting(bracketP, "Brackets");
//	m_renderer->addPainting(movesP, "Moves");
	m_renderer->addPainting(inputP, "Input");
	m_renderer->addPainting(sP, "Simplification");
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	StaircaseDemo demo;
	demo.show();
	QApplication::exec();
}

Staircase::Staircase(std::vector<Number<K>> xs, std::vector<Number<K>> ys):
      m_xs(std::move(xs)), m_ys(std::move(ys)) {
	assert(m_xs.size() + 1 == m_ys.size());
}

bool Staircase::supported_by(const Staircase& other) const {
	auto these_steps = steps();
	auto other_steps = other.steps();

	bool good = true;
	for (const auto& s1 : these_steps) {
		bool found = false;
		for (const auto& s2 : other_steps) {
			if (s1.segment.has_on(s2.segment.source()) && s1.segment.has_on(s2.segment.target())) {
				found = true;
				break;
			}
		}
		if (!found) {
			good = false;
			break;
		}
	}

	return good;
}


bool Staircase::is_valid() const {
	if (m_xs.size() + 1 != m_ys.size()) return false;

	for (int i = 1; i < m_xs.size(); i++) {
		if (m_xs[i-1] > m_xs[i]) {
			return false;
		}
	}

	for (int i = 1; i < m_ys.size(); i++) {
		if (m_ys[i - 1] > m_ys[i]) {
			return false;
		}
	}

	return true;
}

std::vector<Step> Staircase::steps() const {
	std::vector<Step> result;
	for (int i = 0; i < 2 * m_xs.size() - 1; i++) {
		Point<K> p1(m_xs[ceil((i)/2)], m_ys[ceil((i+1)/2)]);
		Point<K> p2(m_xs[ceil((i+1)/2)], m_ys[ceil((i+2)/2)]);

		result.emplace_back(i % 2 == 0, ceil((i + 1) / 2), Segment<K>(p1, p2));
	}
	return result;
}

std::vector<Rectangle<K>> Staircase::moves() {
	std::vector<Rectangle<K>> rects;

	for (int i = 0; i < 2 * m_xs.size() - 2; i++) {
		Point<K> top_left(m_xs[floor(i / 2)], m_ys[floor((i + 1) / 2)]);
		Point<K> bottom_right(m_xs[floor((i / 2) + 1)], m_ys[floor((i + 1) / 2 + 1)]);
		rects.emplace_back(top_left, bottom_right);
	}

	return rects;
}

StaircasePainting::StaircasePainting(const std::shared_ptr<Staircase>& staircase, bool light):
      m_staircase(staircase), m_light(light) {}

void StaircasePainting::paint(GeometryRenderer& renderer) const {
	auto& s = *m_staircase;
	if (m_light) {
		renderer.setStroke(Color(255, 0, 0), 1.5);
	} else {
		renderer.setStroke(Color(0, 0, 0), 2.5);
	}
	renderer.setMode(GeometryRenderer::stroke);

	Ray<K> left(Point<K>(s.m_xs.front(), s.m_ys.front()), Vector<K>(-1.0, 0.0));
	renderer.draw(left);
	for (int i = 0; i < 2 * s.m_xs.size() - 1; i++) {
		Point<K> p1(s.m_xs[ceil((i)/2)], s.m_ys[ceil((i+1)/2)]);
		Point<K> p2(s.m_xs[ceil((i+1)/2)], s.m_ys[ceil((i+2)/2)]);

		renderer.draw(Segment<K>(p1, p2));
	}
	Ray<K> right(Point<K>(s.m_xs.back(), s.m_ys.back()), Vector<K>(11.0, 0.0));
	renderer.draw(right);
}

UniformStaircase::UniformStaircase(int n) : Staircase(std::vector<Number<K>>(n), std::vector<Number<K>>(n+1)) {
	std::iota(m_xs.begin(), m_xs.end(), 0);
	std::iota(m_ys.begin(), m_ys.end(), 0);
}

StaircaseEditable::StaircaseEditable(GeometryWidget* widget, std::shared_ptr<Staircase> staircase, const std::shared_ptr<Staircase>& input)
    : Editable(widget), m_staircase(std::move(staircase)), m_input(input) {}

bool StaircaseEditable::drawHoverHint(Point<Inexact> location, Number<Inexact> radius) const {
	auto closest = closestStep(location, radius);

	if (!closest.has_value()) {
		return false;
	}

	m_widget->setStroke(Color(0, 0, 200), 3.0);
	m_widget->draw(approximate(closest->segment));

	return true;
}

bool StaircaseEditable::startDrag(Point<Inexact> location, Number<Inexact> radius) {
	m_step = closestStep(location, radius);
	if (m_step.has_value()) {
		if (m_step->vertical) {
			QApplication::setOverrideCursor(QCursor(Qt::SizeHorCursor));
		} else {
			QApplication::setOverrideCursor(QCursor(Qt::SizeVerCursor));
		}
	}
	return m_step.has_value();
}

void StaircaseEditable::handleDrag(Point<Inexact> to) const {
	//	*m_point = to;
	if (m_step.has_value()) {
		if (m_step->vertical) {
			Number<K> temp = m_staircase->m_xs[m_step->index];
			m_staircase->m_xs[m_step->index] = to.x();
			if (!m_staircase->is_valid()) {
				m_staircase->m_xs[m_step->index] = temp;
			}
		} else {
			Number<K> temp = m_staircase->m_ys[m_step->index];
			m_staircase->m_ys[m_step->index] = to.y();
			if (!m_staircase->is_valid()) {
				m_staircase->m_ys[m_step->index] = temp;
			}
		}
	}
}

Number<K> snap(const Staircase& s, Number<K> value, bool vertical) {
	const std::vector<Number<K>>* vs;
	if (vertical) {
		vs = &s.m_xs;
	} else {
		vs = &s.m_ys;
	}

	std::optional<Number<K>> closest;
	for (Number<K> v : *vs) {
			if (!closest.has_value() || abs(value - v) < abs(*closest - value)) {
			closest = v;
		}
	}

	return *closest;
}

void StaircaseEditable::endDrag() {
	QApplication::restoreOverrideCursor();
	if (m_step.has_value()) {
		Number<K> v;
		auto& xs = m_staircase->m_xs;
		auto& ys = m_staircase->m_ys;
		if (m_step->vertical) {
			v = xs[m_step->index];
		} else {
			v = ys[m_step->index];
		}
		Number<K> snapped = snap(*m_input, v, m_step->vertical);
		if (m_step->vertical) {
			xs[m_step->index] = snapped;
			if (m_step->index > 0 && xs[m_step->index - 1] == snapped) {
				xs.erase(std::next(xs.begin(), m_step->index));
				ys.erase(std::next(ys.begin(), m_step->index));
			} else if (m_step->index + 1 < xs.size() && xs[m_step->index + 1] == snapped) {
				xs.erase(std::next(xs.begin(), m_step->index));
				ys.erase(std::next(ys.begin(), m_step->index + 1));
			}
		} else {
			ys[m_step->index] = snapped;
			if (m_step->index > 0 && ys[m_step->index - 1] == snapped ) {
				xs.erase(std::next(xs.begin(), m_step->index - 1));
				ys.erase(std::next(ys.begin(), m_step->index));
			} else if (m_step->index + 1 < ys.size() && ys[m_step->index + 1] == snapped) {
				xs.erase(std::next(xs.begin(), m_step->index));
				ys.erase(std::next(ys.begin(), m_step->index));
			}
		}
	}
	m_step = std::nullopt;
	m_widget->repaint();
}

std::optional<Step> StaircaseEditable::closestStep(Point<Inexact> location, Number<K> radius) const {
	Number<K> min_dist = 100000;//std::numeric_limits<double>::infinity();
	std::optional<Step> closest;
	auto steps = m_staircase->steps();
	for (auto step : steps) {
		Number<K> dist = squared_distance(approximate(step.segment), location);
		if (dist < radius * radius && dist < min_dist) {
			min_dist = dist;
			closest = step;
		}
	}

	return closest;
}

GridPainting::GridPainting(const std::shared_ptr<Staircase>& staircase):
      m_staircase(staircase) {}

void GridPainting::paint(GeometryRenderer& renderer) const {
	auto& s = *m_staircase;

	renderer.setStroke(Color(200, 200, 200), 0.5);
	renderer.setMode(GeometryRenderer::stroke);
	for (auto x : s.m_xs) {
		renderer.draw(Line<K>(1, 0, -x));
	}
	for (auto y : s.m_ys) {
		renderer.draw(Line<K>(0, 1, -y));
	}

}

bool supported_by(Segment<K> segment, Segment<K> other) {
	return segment.has_on(other.source()) && segment.has_on(other.target());
}

std::vector<Bracket> brackets(const Staircase& input, const Staircase& simplification) {
	std::vector<Bracket> bs;
	if (!simplification.supported_by(input)) return bs;

	auto sim_steps = simplification.steps();
	auto inp_steps = input.steps();

	int j = -1;

	for (const auto& step : sim_steps) {
		int prev_j = j;

		while (j < 0 || !supported_by(step.segment, inp_steps[j].segment) && j < inp_steps.size()) {
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
				renderer.drawText(text_pos, std::to_string((b.end - b.start - 1) / 2));
			}
			if (m_show_bracket_dimensions->isChecked()) {

			}
		}
	}
}

MovesPainting::MovesPainting(const std::shared_ptr<Staircase>& staircase): m_staircase(staircase) {}

void MovesPainting::paint(GeometryRenderer& renderer) const {
	renderer.setMode(GeometryRenderer::fill);
	renderer.setFill(Color(255, 100, 100));

	auto moves = m_staircase->moves();
	for (const auto& m : moves) {
		Polygon<K> poly;
		poly.push_back(m.vertex(0));
		poly.push_back(m.vertex(1));
		poly.push_back(m.vertex(2));
		poly.push_back(m.vertex(3));
		renderer.draw(poly);
	}
}

void do_greedy_step(Staircase& staircase) {
	auto& xs = staircase.m_xs;
	auto& ys = staircase.m_ys;

	if (xs.size() <= 1) return;

	Number<K> min_area = 1000000;//std::numeric_limits<double>::infinity();
	int min_index = -1;

	for (int i = 0; i < 2 * xs.size() - 2; i++) {
		Point<K> bottom_left(xs[floor(i / 2)], ys[floor((i + 1) / 2)]);
		Point<K> top_right(xs[floor((i / 2) + 1)], ys[floor((i + 1) / 2 + 1)]);
		Rectangle<K> rect(bottom_left, top_right);
		auto area = rect.area();
		if (area < min_area) {
			min_area = area;
			min_index = i;
		}
	}

	bool below = min_index % 2 == 0;

	if (below) {
		xs.erase(std::next(xs.begin(), floor(min_index / 2)));
		ys.erase(std::next(ys.begin(), floor((min_index + 1) / 2 + 1)));
	} else {
		xs.erase(std::next(xs.begin(), floor((min_index / 2) + 1)));
		ys.erase(std::next(ys.begin(), floor((min_index + 1) / 2)));
	}
}