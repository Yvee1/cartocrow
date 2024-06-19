#include "treemap_demo.h"
#include "cartocrow/treemap/ok_hsv_hsl.h"
#include "cartocrow/treemap/parse_tree.h"
#include <QApplication>
#include <QDockWidget>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QToolTip>
#include <QVBoxLayout>
#include <utility>

TreemapDemo::TreemapDemo() {
	setWindowTitle("Treemap");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

	//	std::string filename = "/home/steven/Documents/cartocrow/data/test.tree";
	//	std::fstream fin(filename);
	//	std::string input;
	//	if (fin) {
	//		using Iterator = std::istreambuf_iterator<char>;
	//		input.assign(Iterator(fin), Iterator());
	//	}

//	auto input = "(((100)(1))((((1)(1))((1)(1)))(((1)(1))(((1)(1))(1)))))";
	auto input = "(((100)(1))((((10)(1))((1)(120)))(((50)(1))(((1)(1))(1)))))";
	auto node = parse_tree<Number<K>>(input);

	m_treemap = build_treemap(node);
	auto tmp = std::make_shared<TreemapPainting>(*m_treemap);
	m_renderer->addPainting(tmp, "Treemap");
	auto np = std::make_shared<NodePainting>(*m_treemap, m_selected_node);
	m_renderer->addPainting(np, "Node highlight");

//	auto* dockWidget = new QDockWidget();
//	addDockWidget(Qt::RightDockWidgetArea, dockWidget);
//	auto* vWidget = new QWidget();
//	auto* vLayout = new QVBoxLayout(vWidget);
//	vLayout->setAlignment(Qt::AlignTop);
//	dockWidget->setWidget(vWidget);

	connect(m_renderer, &GeometryWidget::clicked, [this](Point<Inexact> pt) {
		Point<K> pt_k(pt.x(), pt.y());
		auto face = face_at_point(pt_k);
		if (!m_treemap->m_face_to_leaf.contains(face)) {
			if (m_info_box != nullptr) {
				m_info_box->close();
				m_selected_node = std::nullopt;
				m_renderer->repaint();
			}
			m_info_box = nullptr;
			return;
		}
		auto leaf = m_treemap->m_face_to_leaf[face];
		m_selected_node = leaf;
		create_info_box(pt, *m_selected_node);
		m_renderer->repaint();
	});

	connect(m_renderer, &GeometryWidget::panned, [this]() {
		if (m_info_box == nullptr) return;
		auto qp = m_renderer->convertPoint(*m_info_box_position);
		m_info_box->move({static_cast<int>(qp.x()), static_cast<int>(qp.y())});
	});
	connect(m_renderer, &GeometryWidget::zoomed, [this]() {
	  if (m_info_box == nullptr) return;
	  auto qp = m_renderer->convertPoint(*m_info_box_position);
	  m_info_box->move({static_cast<int>(qp.x()), static_cast<int>(qp.y())});
	});
}

void TreemapDemo::create_info_box(Point<Inexact> pt, NPV node) {
	if (m_info_box != nullptr) {
		m_info_box->close();
	}
	m_info_box = new QFrame(m_renderer);
	m_info_box_position = pt;
	auto q_pt = m_renderer->convertPoint(pt);
	m_info_box->move({static_cast<int>(q_pt.x()), static_cast<int>(q_pt.y())});
	m_info_box->resize(120, 160);
	QPalette pal;
	pal.setColor(QPalette::Window, Qt::white);
	m_info_box->setAutoFillBackground(true);
	m_info_box->setPalette(pal);
	m_info_box->show();
	m_info_box->setFrameShape(QFrame::StyledPanel);
	m_info_box->setFrameShadow(QFrame::Raised);
	auto* vLayout = new QVBoxLayout(m_info_box);
	vLayout->setAlignment(Qt::AlignTop);

	auto* info = new QLabel("<h3>Info</h3>");
	vLayout->addWidget(info);
	std::stringstream weight_s;
	auto nv = CGAL::to_double(node->value);
	weight_s.unsetf(std::ios::showpoint);
	if (node->value < 100000) {
		weight_s.precision(std::ceil(std::log10(nv)) + 1);
	} else {
		weight_s.precision(2);
	}
	weight_s << "Weight: " << std::noshowpoint << nv;
	auto* leaf_weight = new QLabel(QString::fromStdString(weight_s.str()));
	vLayout->addWidget(leaf_weight);

	auto* hLayout = new QHBoxLayout();
	auto* dec_weight = new QToolButton();
	dec_weight->setText("-");
	hLayout->addWidget(dec_weight);

	auto rebuild = [node, pt, this]() mutable {
		update_weights(node);
		m_treemap = build_treemap(m_treemap->m_tree);
		auto tmp = std::make_shared<TreemapPainting>(*m_treemap);
		m_renderer->clear();
		m_renderer->addPainting(tmp, "Treemap");
		auto np = std::make_shared<NodePainting>(*m_treemap, m_selected_node);
		m_renderer->addPainting(np, "Node highlight");
		m_renderer->repaint();
		create_info_box(pt, node);
	};

	connect(dec_weight, &QToolButton::clicked, [node, rebuild]() mutable {
		if (node->value <= 1) return;
		node->value -= 1;
		rebuild();
	});
	auto* inc_weight = new QToolButton();
	inc_weight->setText("+");
	hLayout->addWidget(inc_weight);
	connect(inc_weight, &QToolButton::clicked, [node, rebuild]() mutable {
		node->value += 1;
		rebuild();
	});
	vLayout->addLayout(hLayout);
	std::stringstream area_s;
	auto area = abs(CGAL::to_double(m_treemap->node_region(node).area()));
	if (area < 100000) {
		area_s.precision(std::ceil(std::log10(area)) + 1);
	} else {
		area_s.precision(2);
	}
	area_s.unsetf(std::ios::showpoint);
	area_s << "Area: " << area;
	auto* region_area = new QLabel(QString::fromStdString(area_s.str()));
	vLayout->addWidget(region_area);
	auto* close_button = new QPushButton("Close");
	connect(close_button, &QPushButton::clicked, [this](){
		m_info_box->close();
		m_selected_node = std::nullopt;
		m_renderer->repaint();
	});
	auto* parent_button = new QPushButton("Parent");
	connect(parent_button, &QPushButton::clicked, [this, pt](){
		auto parent = (*m_selected_node)->parent.lock();
		if (parent != nullptr) {
			m_selected_node = parent;
			create_info_box(pt, parent);
			m_renderer->repaint();
		}
	});
	vLayout->addWidget(parent_button);
	vLayout->addWidget(close_button);
}

Arrangement<K>::Face_handle TreemapDemo::face_at_point(const Point<K>& point) {
	CGAL::Arr_walk_along_line_point_location<Arrangement<K>> pl(*m_treemap->m_arrangement);

	typedef typename Arrangement<K>::Vertex_const_handle Vertex_const_handle;
	typedef typename Arrangement<K>::Halfedge_const_handle Halfedge_const_handle;
	typedef typename Arrangement<K>::Face_const_handle Face_const_handle;

	const Vertex_const_handle* v;
	const Halfedge_const_handle* e;
	const Face_const_handle* f;

	Face_const_handle face;

	auto obj = pl.locate(point);
	if ((f = boost::get<Face_const_handle>(&obj))) {
		face = *f;
	} else if ((e = boost::get<Halfedge_const_handle>(&obj))) {
		face = (*e)->face();
	} else if ((v = boost::get<Vertex_const_handle>(&obj))) {
		face = (*v)->face();
	} else {
		throw std::runtime_error("Arrangement object is not a vertex, halfedge, or face.");
	}

	return m_treemap->m_arrangement->non_const_handle(face);
}

void TreemapDemo::resizeEvent(QResizeEvent *event) {
	if (m_info_box == nullptr) return;
	auto qp = m_renderer->convertPoint(*m_info_box_position);
	m_info_box->move({static_cast<int>(qp.x()), static_cast<int>(qp.y())});
}

NodePainting::NodePainting(const Treemap& treemap, const std::optional<NPV>& node) : m_node(node), m_treemap(treemap) {}

void NodePainting::paint(GeometryRenderer& renderer) const {
	if (!m_node.has_value()) return;
	auto n = *m_node;
	renderer.setMode(GeometryRenderer::stroke);
	renderer.setStroke({0, 0, 0}, 3.0);
	renderer.draw(m_treemap.node_region(n));
}

TreemapPainting::TreemapPainting(Treemap treemap) : m_treemap(std::move(treemap)) {}

void draw_node_in_hue_range(const Treemap& treemap, const NPV& node, GeometryRenderer& renderer, float lower, float upper, float s = 0.75, float v = 0.95, float padding = 0.3, float shift = 0.2) {
	if (node->is_leaf()) {
		renderer.setMode(GeometryRenderer::fill);
		auto h = lower + (upper - lower) / 2;
		auto rgb = ok_color::okhsv_to_srgb({(h + shift) - static_cast<int>(h + shift), s, v});
		renderer.setFill(Color{static_cast<int>(rgb.r * 255), static_cast<int>(rgb.g * 255), static_cast<int>(rgb.b * 255)});
		renderer.draw(treemap.node_region(node));
		return;
	}

	int n = node->children.size();
	for (int i = 0; i < n; i++) {
		float l = lower + (static_cast<float>(i) + padding) / (static_cast<float>(n) + 2 * padding) * (upper - lower);
		float u = lower + (static_cast<float>(i) + 1 + padding) / (static_cast<float>(n) + 2 * padding) * (upper - lower);
		draw_node_in_hue_range(treemap, node->children[i], renderer, l, u, s, v);
	}
}

void TreemapPainting::paint(GeometryRenderer& renderer) const {
	auto arr = *m_treemap.m_arrangement;

	draw_node_in_hue_range(m_treemap, m_treemap.m_tree, renderer, 0.0, 1.0);

	for (auto eit = arr.edges_begin(); eit != arr.edges_end(); eit++) {
		auto edge = *eit;
		Segment<K> seg(edge.source()->point(), edge.target()->point());
		renderer.setStroke(Color{0, 0, 0}, 1.0);
		renderer.setMode(GeometryRenderer::stroke);
		renderer.draw(seg);
	}
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	TreemapDemo demo;
	demo.show();
	QApplication::exec();
}