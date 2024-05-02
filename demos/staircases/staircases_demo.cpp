#include "staircases_demo.h"
#include <QApplication>
#include <QDockWidget>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <QComboBox>
#include <utility>
#include <random>

StaircaseDemo::StaircaseDemo() {
	// =======================================
	// ============  STAIRCASES ==============
	// =======================================

//		auto input = std::make_shared<UniformStaircase>(20);
	//	auto input = std::make_shared<Staircase>(std::vector<double>({0, 1, 5, 6, 10}), std::vector<double>({0, 3, 4, 6, 8, 10}));
	auto input = std::make_shared<Staircase>(
	    std::vector<Number<K>>({0, 1, 5, 6, 10, 11, 20, 21, 26, 30, 33, 35, 37}),
	    std::vector<Number<K>>({0, 3, 4, 6, 8,  10, 12, 14, 19, 25, 26, 27, 29, 30})
	);
	std::shared_ptr<Staircase> s = std::make_shared<Staircase>(*input);

	// =======================================
	// ========= GEOMETRY WIDGET SETUP =======
	// =======================================
	setWindowTitle("Staircase simplification");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

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
	uniformN->setMaximum(10000);
	auto* uniformNLabel = new QLabel("Number of steps");
	uniformNLabel->setBuddy(uniformN);
	vLayout->addWidget(uniformNLabel);
	vLayout->addWidget(uniformN);

	auto* create_uniform = new QPushButton("Create uniform staircase");
	vLayout->addWidget(create_uniform);

	auto* create_random = new QPushButton("Create random staircase");
	vLayout->addWidget(create_random);

	auto* reset_button = new QPushButton("Reset");
	vLayout->addWidget(reset_button);

	auto* undoRedoSection = new QLabel("<h3>Undo redo</h3>");
	vLayout->addWidget(undoRedoSection);

	auto* undo_button = new QPushButton("Undo");
	vLayout->addWidget(undo_button);

	auto* redo_button = new QPushButton("Redo");
	vLayout->addWidget(redo_button);

	auto* infoSection = new QLabel("<h3>Info</h3>");
	vLayout->addWidget(infoSection);

	auto* show_bracket_complexity = new QCheckBox("Show bracket complexity");
	vLayout->addWidget(show_bracket_complexity);
	show_bracket_complexity->setCheckState(Qt::CheckState::Checked);

	auto* show_bracket_dimensions = new QCheckBox("Show bracket dimension");
//	vLayout->addWidget(show_bracket_dimensions);

	auto* total_bracket_complexity_label = new QLabel();
	vLayout->addWidget(total_bracket_complexity_label);

	auto* num_brackets_label = new QLabel();
	vLayout->addWidget(num_brackets_label);

	auto* l_label = new QLabel();
	vLayout->addWidget(l_label);

	auto* m_label = new QLabel();
	vLayout->addWidget(m_label);

	auto* opt_area_label = new QLabel();
	vLayout->addWidget(opt_area_label);

	auto* simp_area_label = new QLabel();
	vLayout->addWidget(simp_area_label);

	auto* area_ratio_label = new QLabel();
	vLayout->addWidget(area_ratio_label);

	auto set_complexity_text = [input, s, l_label, m_label, total_bracket_complexity_label, num_brackets_label, opt_area_label,
	                                simp_area_label, area_ratio_label]() {
		int in = input->num_of_segments();
		int sn = s->num_of_segments();
		int distributed = (in - sn) / 2;
		int num_brackets = (sn - 1);
	    int l = std::floor(distributed / num_brackets);
		int m = (((double) distributed) / ((double) num_brackets) - l) * (double (sn - 1));
		auto f = [](int sl) { return sl * (sl + 1) / 2; };
		int opt_area = (sn - 1 - m) * f(l) + m * f(l + 1);
		auto simp_brackets = brackets(*input, *s);
		int simp_area = 0;
		for (auto& b : simp_brackets) {
			simp_area += f((b.end - b.start - 1)/2);
		}
		double area_ratio = ((double) simp_area) / ((double) opt_area);
	    total_bracket_complexity_label->setText(QString::fromStdString("Distributed complexity: " + std::to_string(distributed)));
	    num_brackets_label->setText(QString::fromStdString("#Brackets: " + std::to_string(num_brackets)));
	  	l_label->setText(QString::fromStdString("Min. opt. complexity: " + std::to_string(l)));
		m_label->setText(QString::fromStdString("m: " + std::to_string(m)));
		opt_area_label->setText(QString::fromStdString("Opt. area: " + std::to_string(opt_area)));
	    simp_area_label->setText(QString::fromStdString("Simp. area: " + std::to_string(simp_area)));
	    area_ratio_label->setText(QString::fromStdString("Area ratio: " + std::to_string(area_ratio)));
	};

	m_update = [this, set_complexity_text, s, input, undo_button, redo_button](std::optional<std::unique_ptr<Command>> command) {
		if (command.has_value()) {
			(*command)->execute();
			m_command_stack.push(std::move(*command));
			m_redo_stack = {};
		}
	    m_renderer->repaint();
		set_complexity_text();

		undo_button->setEnabled(!m_command_stack.empty());
	    redo_button->setEnabled(!m_redo_stack.empty());
	};

	auto* greedySection = new QLabel("<h3>Greedy</h3>");
	vLayout->addWidget(greedySection);

	auto* greedy_strategy_label = new QLabel("Greedy strategy");
	auto* greedy_strategy = new QComboBox();
	vLayout->addWidget(greedy_strategy_label);
	vLayout->addWidget(greedy_strategy);
	greedy_strategy->addItem("First");
	greedy_strategy->addItem("Random");
	greedy_strategy->addItem("Top");
	greedy_strategy->addItem("Bottom");

	auto* greedy_cost_label = new QLabel("Greedy cost");
	auto* greedy_cost = new QComboBox();
	vLayout->addWidget(greedy_cost_label);
	vLayout->addWidget(greedy_cost);
	greedy_cost->addItem("Min. area");
	greedy_cost->addItem("Min. bracket complexity");

	auto* greedy_step = new QPushButton("Step");
	vLayout->addWidget(greedy_step);

	auto* greedy_step_10 = new QPushButton("Step x 10");
	vLayout->addWidget(greedy_step_10);

	auto* greedy_step_100 = new QPushButton("Step x 100");
	vLayout->addWidget(greedy_step_100);

	// =========== EVENT HANDLERS ============
	connect(reset_button, &QPushButton::clicked, [s, input, this] {
		s->m_xs = input->m_xs;
		s->m_ys = input->m_ys;
		m_command_stack = {};
		m_update(std::nullopt);
	});
	connect(create_uniform, &QPushButton::clicked, [uniformN, s, input, this] {
		auto uni = UniformStaircase(uniformN->value());
		input->m_xs = uni.m_xs;
	  	input->m_ys = uni.m_ys;
	    s->m_xs = input->m_xs;
	    s->m_ys = input->m_ys;
		m_update(std::nullopt);
	});
	connect(create_random, &QPushButton::clicked, [uniformN, s, input, this] {
	    std::exponential_distribution dist(1.0);

		std::vector<Number<K>> xs;
	    std::vector<Number<K>> ys;
		xs.push_back(0);
	    ys.push_back(0);
		for (int i = 1; i < uniformN->value(); i++) {
			xs.push_back(xs.back() + dist(m_gen));
			ys.push_back(ys.back() + dist(m_gen));
		}
	  	ys.push_back(ys.back() + dist(m_gen));
		auto stairs = Staircase(xs, ys);

		input->m_xs = xs;
		input->m_ys = ys;
		s->m_xs = input->m_xs;
		s->m_ys = input->m_ys;
		m_update(std::nullopt);
	});
	connect(undo_button, &QPushButton::clicked, [this] {
		auto cmd = std::move(m_command_stack.top());
		cmd->undo();
		m_command_stack.pop();
		m_redo_stack.push(std::move(cmd));
		m_update(std::nullopt);
	});
	connect(redo_button, &QPushButton::clicked, [this] {
		auto cmd = std::move(m_redo_stack.top());
		cmd->execute();
		m_redo_stack.pop();
		m_command_stack.push(std::move(cmd));
		m_update(std::nullopt);
	});
	connect(show_bracket_complexity, &QCheckBox::stateChanged, [this]{ m_renderer->repaint(); });
//	connect(show_bracket_dimensions, &QCheckBox::stateChanged, [this]{ m_renderer->repaint(); });

	connect(greedy_strategy, &QComboBox::currentTextChanged, [this] { m_renderer->repaint(); });
	connect(greedy_cost, &QComboBox::currentTextChanged, [this] { m_renderer->repaint(); });
	connect(greedy_step, &QPushButton::clicked, [s, input, this, greedy_strategy, greedy_cost] {
		std::shared_ptr<Staircase> t = s;
		GreedyCost cost;
		int gci = greedy_cost->currentIndex();
		if (gci == 0) {
			cost = greedy_area;
		} else {
			cost = greedy_bracket;
		}
		auto c = greedy_contraction(t, *input, static_cast<GreedyStrategy>(greedy_strategy->currentIndex()), cost, m_gen);
		if (c.has_value()) {
			m_update(std::move(*c));
		}
	});
	connect(greedy_step_100, &QPushButton::clicked, [s, input, this, greedy_strategy, greedy_cost] {
		std::shared_ptr<Staircase> t = s;
		GreedyCost cost;
		int gci = greedy_cost->currentIndex();
		if (gci == 0) {
			cost = greedy_area;
		} else {
			cost = greedy_bracket;
		}
		for (int i = 0; i < 100; i++) {
			auto c = greedy_contraction(t, *input, static_cast<GreedyStrategy>(greedy_strategy->currentIndex()), cost, m_gen);
			if (c.has_value()) {
				(*c)->execute();
				m_command_stack.push(std::move(*c));
			}
		}
		m_update(std::nullopt);
	});
	connect(greedy_step_10, &QPushButton::clicked, [s, input, this, greedy_strategy, greedy_cost] {
		std::shared_ptr<Staircase> t = s;
		GreedyCost cost;
		int gci = greedy_cost->currentIndex();
		if (gci == 0) {
			cost = greedy_area;
		} else {
			cost = greedy_bracket;
		}
		for (int i = 0; i < 10; i++) {
			auto c = greedy_contraction(t, *input, static_cast<GreedyStrategy>(greedy_strategy->currentIndex()), cost, m_gen);
			if (c.has_value()) {
				(*c)->execute();
				m_command_stack.push(std::move(*c));
			}
		}
		m_update(std::nullopt);
	});

	// =======================================
	// ============== PAINTINGS ==============
	// =======================================
	auto gridP = std::make_shared<GridPainting>(input);
	auto bracketP = std::make_shared<BracketPainting>(input, s, show_bracket_complexity, show_bracket_dimensions);
	auto movesP = std::make_shared<MovesPainting>(s);
	auto minMovesP = std::make_shared<MinMovesPainting>(s, input, greedy_cost);
	auto inputP = std::make_shared<StaircasePainting>(input, true);
	auto sP = std::make_shared<StaircasePainting>(s, false);
	m_renderer->addPainting(gridP, "Grid");
	m_renderer->addPainting(bracketP, "Brackets");
	m_renderer->addPainting(movesP, "Moves");
	m_renderer->addPainting(minMovesP, "Min. moves");
	m_renderer->addPainting(inputP, "Input");
	m_renderer->addPainting(sP, "Simplification");

	// =======================================
	// ============== EDITABLES ==============
	// =======================================
	m_renderer->m_editables.push_back(std::make_unique<StaircaseEditable>(m_renderer, s, input, m_update));

	m_update(std::nullopt);
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	StaircaseDemo demo;
	demo.show();
	QApplication::exec();
}

StaircaseEditable::StaircaseEditable(GeometryWidget* widget, std::shared_ptr<Staircase> staircase,
                                     const std::shared_ptr<Staircase>& input,
                                     std::function<void(std::optional<std::unique_ptr<Command>>)> update)
    : Editable(widget), m_staircase(std::move(staircase)), m_input(input), m_update(std::move(update)) {}

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
	m_edge = closestStep(location, radius);
	if (m_edge.has_value()) {
		if (m_edge->vertical) {
			QApplication::setOverrideCursor(QCursor(Qt::SizeHorCursor));
			m_start_position = m_staircase->m_xs[m_edge->index];
		} else {
			QApplication::setOverrideCursor(QCursor(Qt::SizeVerCursor));
			m_start_position = m_staircase->m_ys[m_edge->index];
		}
	}
	return m_edge.has_value();
}

void StaircaseEditable::handleDrag(Point<Inexact> to) const {
	if (m_edge.has_value()) {
		if (m_edge->vertical) {
			Number<K> temp = m_staircase->m_xs[m_edge->index];
			m_staircase->m_xs[m_edge->index] = to.x();
			if (!m_staircase->is_valid()) {
				m_staircase->m_xs[m_edge->index] = temp;
			}
		} else {
			Number<K> temp = m_staircase->m_ys[m_edge->index];
			m_staircase->m_ys[m_edge->index] = to.y();
			if (!m_staircase->is_valid()) {
				m_staircase->m_ys[m_edge->index] = temp;
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
	if (m_edge.has_value()) {
		Number<K> v;
		auto& xs = m_staircase->m_xs;
		auto& ys = m_staircase->m_ys;
		if (m_edge->vertical) {
			v = xs[m_edge->index];
		} else {
			v = ys[m_edge->index];
		}
		Number<K> snapped = snap(*m_input, v, m_edge->vertical);

		std::unique_ptr<Command> command = move_or_contract(m_staircase, *m_edge, *m_start_position, snapped);
		m_update(std::move(command));
	}
	m_edge = std::nullopt;
	m_start_position = std::nullopt;
}

std::optional<Edge> StaircaseEditable::closestStep(Point<Inexact> location, Number<K> radius) const {
	Number<K> min_dist = 100000;//std::numeric_limits<double>::infinity();
	std::optional<Edge> closest;
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

MinMovesPainting::MinMovesPainting(const std::shared_ptr<Staircase>& staircase, const std::shared_ptr<Staircase>& input,
                                   QComboBox* cost_qt): m_simplification(staircase), m_input(input), m_cost_qt(std::move(cost_qt)) {}

void MinMovesPainting::paint(GeometryRenderer& renderer) const {
	if (!m_simplification->supported_by(*m_input)) return;

	renderer.setMode(GeometryRenderer::fill);
	renderer.setFill(Color(100, 255, 100));

	GreedyCost cost;
	int gci = m_cost_qt->currentIndex();
	if (gci == 0) {
		cost = greedy_area;
	} else {
		cost = greedy_bracket;
	}

	auto min_moves = min_cost_moves(*m_simplification, *m_input, cost);
	for (const auto& m : min_moves) {
		auto& r = m.rectangle;
		Polygon<K> poly;
		poly.push_back(r.vertex(0));
		poly.push_back(r.vertex(1));
		poly.push_back(r.vertex(2));
		poly.push_back(r.vertex(3));
		renderer.draw(poly);
	}
}
