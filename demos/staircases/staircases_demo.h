#ifndef CARTOCROW_STAIRCASES_H
#define CARTOCROW_STAIRCASES_H

#include "types.h"
#include "staircase.h"
#include "brackets.h"
#include "move.h"
#include "greedy.h"
#include <QCheckBox>
#include <QMainWindow>
#include <random>
#include <QComboBox>
#include <QMainWindow>

class GridPainting : public GeometryPainting {
  public:
	GridPainting(const std::shared_ptr<Staircase>& staircase);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_staircase;
};

class MinMovesPainting : public GeometryPainting {
  public:
	MinMovesPainting(const std::shared_ptr<Staircase>& staircase, const std::shared_ptr<Staircase>& input, QComboBox* cost_qt);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const std::shared_ptr<Staircase> m_simplification;
	const std::shared_ptr<Staircase> m_input;
	QComboBox* m_cost_qt;
};

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
	std::stack<std::unique_ptr<Command>> m_redo_stack;
	std::mt19937 m_gen;
};

#endif //CARTOCROW_STAIRCASES_H
