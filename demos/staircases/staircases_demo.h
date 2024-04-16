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
#include <QMainWindow>


using namespace cartocrow;
using namespace cartocrow::renderer;

typedef Inexact K;

#include <QMainWindow>

class StaircaseDemo : public QMainWindow {
	Q_OBJECT

  public:
	StaircaseDemo();

  private:
	GeometryWidget* m_renderer;

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

	std::vector<Step> steps() {
		std::vector<Step> result;
		for (int i = 0; i < 2 * m_xs.size() - 1; i++) {
			Point<K> p1(m_xs[ceil((i)/2)], m_ys[ceil((i+1)/2)]);
			Point<K> p2(m_xs[ceil((i+1)/2)], m_ys[ceil((i+2)/2)]);

			result.emplace_back(i % 2 == 0, ceil((i + 1) / 2), Segment<K>(p1, p2));
		}
		return result;
	}

	bool is_valid() {
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
	bool drawHoverHint(Point<K> location, Number<K> radius) const override;
	bool startDrag(Point<K> location, Number<K> radius) override;
	void handleDrag(Point<K> to) const override;
	void endDrag() override;

  private:
	/// Checks if the location is within a circle with the given radius
	/// around the point.
	std::optional<Step> closestStep(Point<K> location, Number<K> radius) const;
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
#endif //CARTOCROW_STAIRCASES_H
