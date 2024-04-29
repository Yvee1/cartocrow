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

GreedyCost greedy_area = [](const Staircase& s, const Staircase& input, const MoveBox& box) {
	return box.rectangle.area();
};

GreedyCost greedy_bracket = [](const Staircase& s, const Staircase& input, const MoveBox& box) {
	auto bs = brackets(input, s);

    Bracket l = bs[box.index];
	Bracket m = bs[box.index + 1];
    Bracket r = bs[box.index + 2];

	return l.complexity() + m.complexity() + r.complexity();
};

StaircaseDemo::StaircaseDemo() {
	// =======================================
	// ============  STAIRCASES ==============
	// =======================================

		auto input = std::make_shared<UniformStaircase>(20);
	//	auto input = std::make_shared<Staircase>(std::vector<double>({0, 1, 5, 6, 10}), std::vector<double>({0, 3, 4, 6, 8, 10}));
//	auto input = std::make_shared<Staircase>(
//	    std::vector<Number<K>>({0, 1, 5, 6, 10, 11, 20, 21, 26, 30, 33, 35, 37}),
//	    std::vector<Number<K>>({0, 3, 4, 6, 8,  10, 12, 14, 19, 25, 26, 27, 29, 30})
//	);
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
	auto* uniformNLabel = new QLabel("Uniform n");
	uniformNLabel->setBuddy(uniformN);
	vLayout->addWidget(uniformNLabel);
	vLayout->addWidget(uniformN);

	auto* create_uniform = new QPushButton("Create uniform staircase");
	vLayout->addWidget(create_uniform);

	auto* reset_button = new QPushButton("Reset");
	vLayout->addWidget(reset_button);

	auto* undoRedoSection = new QLabel("<h3>Undo redo</h3>");
	vLayout->addWidget(undoRedoSection);

	auto* undo_button = new QPushButton("Undo");
	vLayout->addWidget(undo_button);

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

	m_update = [this, set_complexity_text, s, input, undo_button](std::optional<std::unique_ptr<Command>> command) {
		if (command.has_value()) {
			(*command)->execute();
			m_command_stack.push(std::move(*command));
		}
	    m_renderer->repaint();
		set_complexity_text();

		undo_button->setEnabled(!m_command_stack.empty());
	};

	auto* greedySection = new QLabel("<h3>Greedy</h3>");
	vLayout->addWidget(greedySection);

	auto* greedy_strategy_label = new QLabel("Greedy strategy");
	auto* greedy_strategy = new QComboBox();
	vLayout->addWidget(greedy_strategy_label);
	vLayout->addWidget(greedy_strategy);
	greedy_strategy->addItem("Top");
	greedy_strategy->addItem("Bottom");
	greedy_strategy->addItem("First");
	greedy_strategy->addItem("Random");

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
	connect(undo_button, &QPushButton::clicked, [this] {
		m_command_stack.top()->undo();
		m_command_stack.pop();
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

std::vector<Edge> Staircase::steps() const {
	std::vector<Edge> result;
	for (int i = 0; i < 2 * m_xs.size() - 1; i++) {
		Point<K> p1(m_xs[ceil((i)/2)], m_ys[ceil((i+1)/2)]);
		Point<K> p2(m_xs[ceil((i+1)/2)], m_ys[ceil((i+2)/2)]);

		result.emplace_back(i % 2 == 0, ceil((i + 1) / 2), Segment<K>(p1, p2));
	}
	return result;
}

std::vector<MoveBox> Staircase::moves() const {
	std::vector<MoveBox> boxes;
	for (int i = 0; i < 2 * m_xs.size() - 2; i++) {
		Point<K> top_left(m_xs[floor(i / 2)], m_ys[floor((i + 1) / 2)]);
		Point<K> bottom_right(m_xs[floor((i / 2) + 1)], m_ys[floor((i + 1) / 2 + 1)]);
		boxes.emplace_back(i, Rectangle<K>(top_left, bottom_right));
	}
	return boxes;
}

MoveBox Staircase::move(int i) const {
	Point<K> top_left(m_xs[floor(i / 2)], m_ys[floor((i + 1) / 2)]);
	Point<K> bottom_right(m_xs[floor((i / 2) + 1)], m_ys[floor((i + 1) / 2 + 1)]);
	return { i, Rectangle<K>(top_left, bottom_right) };
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

bool supported_by(Segment<K> segment, Segment<K> other) {
	return segment.has_on(other.source()) && segment.has_on(other.target());
}

std::vector<Bracket> brackets(const Staircase& input, const Staircase& simplification) {
	std::vector<Bracket> bs;

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

//Bracket bracket(const Staircase& input, const Staircase& simplification, int i) {
//	return brackets(input, simp)
//}

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

MovesPainting::MovesPainting(const std::shared_ptr<Staircase>& staircase): m_staircase(staircase) {}

void MovesPainting::paint(GeometryRenderer& renderer) const {
	renderer.setMode(GeometryRenderer::fill);
	renderer.setFill(Color(255, 100, 100));

	auto moves = m_staircase->moves();
	for (const auto& m : moves) {
		auto& r = m.rectangle;
		Polygon<K> poly;
		poly.push_back(r.vertex(0));
		poly.push_back(r.vertex(1));
		poly.push_back(r.vertex(2));
		poly.push_back(r.vertex(3));
		renderer.draw(poly);
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

std::vector<MoveBox> min_cost_moves(const Staircase& simp, const Staircase& input, const GreedyCost& cost) {
	std::vector<MoveBox> moves = simp.moves();

	std::optional<Number<K>> min_cost;
	for (const auto& m : moves) {
		auto c = cost(simp, input, m);
		if (!min_cost.has_value() || c < *min_cost) {
			min_cost = c;
		}
	}

	if (!min_cost.has_value())
		return {};

	std::vector<MoveBox> minimum;
	for (const auto& m : moves) {
		if (cost(simp, input, m) == min_cost) {
			minimum.push_back(m);
		}
	}

	return minimum;
}


std::optional<std::unique_ptr<Contraction>> greedy_contraction(std::shared_ptr<Staircase>& simp,
                                                               const Staircase& input,
                                                               GreedyStrategy strategy,
															   GreedyCost cost,
                                                               std::mt19937& gen) {
	auto& xs = simp->m_xs;
	auto& ys = simp->m_ys;

	if (xs.size() <= 1) return std::nullopt;

	if (strategy != GreedyStrategy::RANDOM) {
		std::optional<Number<K>> min_cost;
		std::optional<MoveBox> min_move;

		for (int i = 0; i < 2 * xs.size() - 2; i++) {
			bool allowed;
			if (strategy == GreedyStrategy::FIRST) {
				allowed = true;
			} else if (strategy == GreedyStrategy::TOP) {
				allowed = i % 2 != 0;
			} else if (strategy == GreedyStrategy::BOTTOM) {
				allowed = i % 2 == 0;
			}
			if (!allowed) {
				continue;
			}

			auto move = simp->move(i);
			Number<K> c = cost(*simp, input, move);
			if (!min_cost.has_value() || c < *min_cost) {
				min_cost = c;
				min_move = move;
			}
		}

		if (!min_move.has_value()) return std::nullopt;
		return std::make_unique<Contraction>(simp, *min_move);
	} else {
		auto minimum = min_cost_moves(*simp, input, cost);
		std::uniform_int_distribution<int> dist(0, minimum.size() - 1);
		auto move = minimum[dist(gen)];
		return std::make_unique<Contraction>(simp, move);
	}

}

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
