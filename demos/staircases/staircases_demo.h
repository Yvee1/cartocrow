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

#ifndef CARTOCROW_STAIRCASES_H
#define CARTOCROW_STAIRCASES_H

#include "cartocrow/core/core.h"
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <QCheckBox>
#include <QMainWindow>


using namespace cartocrow;
using namespace cartocrow::renderer;

template <class K> using Rectangle = CGAL::Iso_rectangle_2<K>;

typedef Inexact K;

#include <QMainWindow>

class StaircaseDemo : public QMainWindow {
	Q_OBJECT

  public:
	StaircaseDemo();

  private:
	GeometryWidget* m_renderer;
	std::function<void()> m_update;
};

class OrthoPolygon {
	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;
};

struct Step {
	bool vertical;
	int index;
	Segment<K> segment;
};

class Staircase {
  public:
	Staircase(std::vector<Number<K>> xs, std::vector<Number<K>> ys);

	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;

	[[nodiscard]] std::vector<Step> steps() const;

	[[nodiscard]] bool is_valid() const;

	[[nodiscard]] bool supported_by(const Staircase& other) const;

	std::vector<CGAL::Iso_rectangle_2<K>> moves();

  private:
};

class UniformStaircase : public Staircase {
  public:
	/// Create a uniform staircase with n steps (vertical segments)
	UniformStaircase(int n);
};

class StaircasePainting : public GeometryPainting {
  public:
	StaircasePainting(const std::shared_ptr<Staircase>& staircase, bool light);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
	bool m_light;
};

class StaircaseEditable : public GeometryWidget::Editable {
  public:
	StaircaseEditable(GeometryWidget* widget, std::shared_ptr<Staircase> staircase, const std::shared_ptr<Staircase>& input);
	bool drawHoverHint(Point<Inexact> location, Number<Inexact> radius) const override;
	bool startDrag(Point<Inexact> location, Number<Inexact> radius) override;
	void handleDrag(Point<Inexact> to) const override;
	void endDrag() override;

  private:
	/// Checks if the location is within a circle with the given radius
	/// around the point.
	std::optional<Step> closestStep(Point<Inexact> location, Number<K> radius) const;
	/// The staircase that we are editing.
	std::shared_ptr<Staircase> m_staircase;
	/// The input staircase.
	const std::shared_ptr<Staircase> m_input;
	/// The step that we are dragging
	std::optional<Step> m_step;
};

class GridPainting : public GeometryPainting {
  public:
	GridPainting(const std::shared_ptr<Staircase>& staircase);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
};

bool supported_by(Segment<K> segment, Segment<K> other);

struct Bracket {
	int start;
	int end;

	std::optional<Polygon<K>> polygon(const Staircase& input) const {
		bool vertical = start % 2 == 0;
		if (start + 1 == end) return std::nullopt;

		std::vector<Point<K>> pts;
		pts.emplace_back(input.m_xs[ceil((start + 1)/2)], input.m_ys[ceil((start+2)/2)]);
		for (int i = start + 1; i < end; i++) {
			pts.emplace_back(input.m_xs[ceil((i+1)/2)], input.m_ys[ceil((i+2)/2)]);
		}
		if (vertical) {
			pts.emplace_back(pts.front().x(), pts.back().y());
		} else {
			pts.emplace_back(pts.back().x(), pts.front().y());
		}

		return Polygon<K>(pts.begin(), pts.end());
	}
};

std::vector<Bracket> brackets(const Staircase& input, const Staircase& simplification);

class BracketPainting : public GeometryPainting {
  public:
	BracketPainting(const std::shared_ptr<Staircase>& input,
	                const std::shared_ptr<Staircase>& simplification,
	                QCheckBox* show_bracket_complexity,
					QCheckBox* show_bracket_dimensions);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_input;
	const std::shared_ptr<Staircase> m_simplification;
	QCheckBox* m_show_bracket_complexity;
	QCheckBox* m_show_bracket_dimensions;

};

class MovesPainting : public GeometryPainting {
  public:
	MovesPainting(const std::shared_ptr<Staircase>& staircase);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
	const std::vector<Polygon<K>> m_polys;
};

void do_greedy_step(Staircase& staircase);

#endif //CARTOCROW_STAIRCASES_H
