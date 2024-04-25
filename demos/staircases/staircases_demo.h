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

class OrthoPolygon {
	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;
};

struct Edge {
	bool vertical;
	int index;
	Segment<K> segment;
};

struct MoveBox {
	int index;
	Rectangle<K> rectangle;
};

class Staircase {
  public:
	Staircase(std::vector<Number<K>> xs, std::vector<Number<K>> ys);

	std::vector<Number<K>> m_xs;
	std::vector<Number<K>> m_ys;

	[[nodiscard]] std::vector<Edge> steps() const;

	[[nodiscard]] bool is_valid() const;

	[[nodiscard]] bool supported_by(const Staircase& other) const;

	MoveBox move(int i) const;

	std::vector<MoveBox> moves() const;

	[[nodiscard]] size_t num_of_segments() const {
		return m_xs.size() + m_ys.size();
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

class Command {
  public:
	virtual void execute() = 0;
	virtual void undo() = 0;
	virtual ~Command() = default;
};

class Contraction : public Command {
  public:
	Contraction(std::shared_ptr<Staircase> staircase, MoveBox box);
	void execute() override;
	void undo() override;

  private:
	std::shared_ptr<Staircase> m_staircase;
	MoveBox m_box;
	std::optional<Number<K>> m_erased_x;
	std::optional<Number<K>> m_erased_y;
};

class EdgeMove : public Command {
  public:
	EdgeMove(std::shared_ptr<Staircase> staircase, Edge edge, Number<K> start_pos, Number<K> new_pos);
	void execute() override;
	void undo() override;

  private:
	std::shared_ptr<Staircase> m_staircase;
	Edge m_edge;
	Number<K> m_new_pos;
	Number<K> m_start_pos;
};

std::optional<std::unique_ptr<Contraction>> greedy_contraction(std::shared_ptr<Staircase>& staircase);

std::unique_ptr<Command> move_or_contract(const std::shared_ptr<Staircase>& staircase, Edge edge, Number<K> start_pos, Number<K> new_pos);

class StaircaseEditable : public GeometryWidget::Editable {
  public:
	StaircaseEditable(GeometryWidget* widget,
	                  std::shared_ptr<Staircase> staircase,
	                  const std::shared_ptr<Staircase>& input,
	                  std::function<void(std::optional<std::unique_ptr<Command>>)> update);
	bool drawHoverHint(Point<Inexact> location, Number<Inexact> radius) const override;
	bool startDrag(Point<Inexact> location, Number<Inexact> radius) override;
	void handleDrag(Point<Inexact> to) const override;
	void endDrag() override;

  private:
	/// Checks if the location is within a circle with the given radius
	/// around the point.
	std::optional<Edge> closestStep(Point<Inexact> location, Number<K> radius) const;
	/// The staircase that we are editing.
	std::shared_ptr<Staircase> m_staircase;
	/// The input staircase.
	const std::shared_ptr<Staircase> m_input;
	/// The step that we are dragging
	std::optional<Edge> m_edge;
	/// Update function called after dragging ends.
	std::function<void(std::unique_ptr<Command>)> m_update;
	/// Start position of edge
	std::optional<Number<K>> m_start_position;
};

class StaircaseDemo : public QMainWindow {
	Q_OBJECT

  public:
	StaircaseDemo();

  private:
	GeometryWidget* m_renderer;
	std::function<void(std::optional<std::unique_ptr<Command>>)> m_update;
	std::stack<std::unique_ptr<Command>> m_command_stack;
};

#endif //CARTOCROW_STAIRCASES_H
