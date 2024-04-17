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
#include <utility>

StaircaseDemo::StaircaseDemo() {
	setWindowTitle("Staircase simplification");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

	auto input = std::make_shared<UniformStaircase>(20);
//	auto input = std::make_shared<Staircase>(std::vector<double>({0, 1, 5, 6, 10}), std::vector<double>({0, 3, 4, 6, 8, 10}));
	auto gridP = std::make_shared<GridPainting>(input);

	auto s = std::make_shared<Staircase>(*input);
	auto inputP = std::make_shared<StaircasePainting>(input, true);
	auto sP = std::make_shared<StaircasePainting>(s, false);
	m_renderer->addPainting(gridP, "Grid");
	m_renderer->addPainting(inputP, "Input");
	m_renderer->addPainting(sP, "Simplification");
	m_renderer->m_editables.push_back(std::make_unique<StaircaseEditable>(m_renderer, s, input));
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

StaircasePainting::StaircasePainting(const std::shared_ptr<Staircase>& staircase, bool light):
      m_staircase(staircase), m_light(light) {}

void StaircasePainting::paint(GeometryRenderer& renderer) const {
	auto& s = *m_staircase;
	if (m_light) {
		renderer.setStroke(Color(150, 150, 150), 2.0);
	} else {
		renderer.setStroke(Color(0, 0, 0), 2.0);
	}

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

bool StaircaseEditable::drawHoverHint(Point<K> location, Number<K> radius) const {
	auto closest = closestStep(location, radius);

	if (!closest.has_value()) {
		return false;
	}

	m_widget->setStroke(Color(0, 0, 200), 3.0);
	m_widget->draw(closest->segment);

	return true;
}

bool StaircaseEditable::startDrag(Point<K> location, Number<K> radius) {
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

void StaircaseEditable::handleDrag(Point<K> to) const {
	//	*m_point = to;
	if (m_step.has_value()) {
		if (m_step->vertical) {
			double temp = m_staircase->m_xs[m_step->index];
			m_staircase->m_xs[m_step->index] = to.x();
			if (!m_staircase->is_valid()) {
				m_staircase->m_xs[m_step->index] = temp;
			}
		} else {
			double temp = m_staircase->m_ys[m_step->index];
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
	for (double v : *vs) {
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
}

std::optional<Step> StaircaseEditable::closestStep(Point<K> location, Number<K> radius) const {
	double min_dist = std::numeric_limits<double>::infinity();
	std::optional<Step> closest;
	auto steps = m_staircase->steps();
	for (auto step : steps) {
		double dist = squared_distance(step.segment, location);
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

	renderer.setStroke(Color(200, 200, 200), 1.0);
	for (auto x : s.m_xs) {
		renderer.draw(Line<K>(1, 0, -x));
	}
	for (auto y : s.m_ys) {
		renderer.draw(Line<K>(0, 1, -y));
	}

}