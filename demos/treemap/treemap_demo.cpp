#include "treemap_demo.h"
#include "cartocrow/treemap/aspect_ratio.h"
#include "cartocrow/treemap/convex.h"
#include "cartocrow/treemap/orthoconvex.h"
#include "cartocrow/treemap/parse_csv_to_tree.h"
#include "cartocrow/treemap/parse_tree.h"
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

TreemapDemo::TreemapDemo() {
	setWindowTitle("Treemap");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	m_renderer->setMinZoom(0.01);
	m_renderer->setMaxZoom(1000.0);

	auto* dockWidget = new QDockWidget();
	addDockWidget(Qt::RightDockWidgetArea, dockWidget);
	auto* vWidget = new QWidget();
	auto* vLayout = new QVBoxLayout(vWidget);
	vLayout->setAlignment(Qt::AlignTop);
	dockWidget->setWidget(vWidget);

	auto* treemapOptions = new QLabel("<h3>Treemap type</h3>");
	vLayout->addWidget(treemapOptions);
	auto* treemapTypeLabel = new QLabel("Treemap type");
	auto* treemapType = new QComboBox();
	treemapType->addItem("Orthoconvex");
	treemapType->addItem("Convex");
	vLayout->addWidget(treemapTypeLabel);
	vLayout->addWidget(treemapType);
	m_treemap_builder = [treemapType](NP<Named>& tree, const Rectangle<K>& rect, NodeWeight<Named> w) {
		if (treemapType->currentText() == "Orthoconvex") {
			return orthoconvex_treemap(tree, rect, w);
		} else if (treemapType->currentText() == "Convex") {
			return convex_treemap(tree, rect, w);
		} else {
			throw std::runtime_error("Treemap type " + treemapType->currentText().toStdString() + " has not been implemented.");
		}
	};

	auto* basicOptions = new QLabel("<h3>Input</h3>");
	vLayout->addWidget(basicOptions);
	auto* fileSelector = new QPushButton("Select file");
	vLayout->addWidget(fileSelector);

	auto* timeStepInput = new QSpinBox();
	timeStepInput->setValue(0);
	timeStepInput->setMaximum(100);
	auto* timeStepLabel = new QLabel("Time step");
	timeStepLabel->setBuddy(timeStepInput);
	vLayout->addWidget(timeStepLabel);
	vLayout->addWidget(timeStepInput);

	// Read a csv containing sequences of weights in a hierarchy into a tree
//	load_file("data/wb-SM.POP.NETM-net-migration.data");
//	load_file("data/convex_test_correct.data");
	load_file("data/convex_hierarchy.data");
//	load_file("data/convex_test_breaks.data");
//	load_file("data/convex_test_tiny.data");

	connect(m_renderer, &GeometryWidget::clicked, [this](Point<Inexact> pt) {
		Point<K> pt_k(pt.x(), pt.y());
		auto face = face_at_point(pt_k);
		if (!m_treemap->m_faceToLeaf.contains(face)) {
			if (m_info_box != nullptr) {
				m_info_box->close();
				m_selected_node = std::nullopt;
				m_renderer->repaint();
			}
			m_info_box = nullptr;
			return;
		}
		auto leaf = m_treemap->m_faceToLeaf[face];
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
	connect(timeStepInput, QOverload<int>::of(&QSpinBox::valueChanged), [this, timeStepInput]() {
	    if (timeStepInput->value() >= m_treemap->m_tree->value.weight.size()) {
			timeStepInput->setValue(m_timestep);
		    return;
	    }
		m_timestep = timeStepInput->value();
		m_treemap = m_treemap_builder(m_treemap->m_tree, m_rect, timestep_weights(m_timestep));
		updated_treemap();
	});
	connect(fileSelector, &QPushButton::clicked, [this, fileSelector]() {
		QString start_dir;
//		if (m_dir.has_value() && m_dir != "") {
//			start_dir = QString::fromStdString(*m_dir);
//		} else {
//			start_dir = ".";
	  start_dir = "data/";
//		}

		std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select treemap data file"), start_dir).toStdString();

		if (filePath == "") return;
		load_file(filePath);
		fileSelector->setText(QString::fromStdString(filePath.filename()));
	});
	connect(treemapType, &QComboBox::currentTextChanged, [this, treemapType]() {
		m_treemap = m_treemap_builder(m_treemap->m_tree, m_rect, timestep_weights(m_timestep));
		m_tmp = std::make_shared<TreemapPainting<Named>>(*m_treemap, *m_treemap);
		updated_treemap();
	});
}

void TreemapDemo::clear_info_box() {
	if (m_info_box != nullptr) {
		m_info_box->close();
	}
	m_info_box = nullptr;
	m_info_box_position = std::nullopt;
}

void TreemapDemo::updated_treemap() {
	m_renderer->clear();
	m_tmp = std::make_shared<TreemapPainting<Named>>(*m_treemap, m_tmp->m_initial_treemap);
	m_renderer->addPainting(m_tmp, "Treemap");
	auto np = std::make_shared<NodePainting<Named>>(*m_treemap, m_selected_node);
	m_renderer->addPainting(np, "Node highlight");
	m_renderer->repaint();
	if (m_info_box != nullptr) {
		create_info_box(*m_info_box_position, *m_selected_node);
	}
}

void TreemapDemo::load_file(const std::filesystem::path& filePath) {
	m_timestep = 0;
	m_selected_node = std::nullopt;
	clear_info_box();

	if (filePath.empty()) return;
	std::ifstream t(filePath, std::ios_base::in);
	if (!t.good()) {
		throw std::runtime_error("Failed to read input file");
	}
	std::stringstream buffer;
	buffer << t.rdbuf();
	auto multi_tree = parse_csv_to_tree(buffer.str());
	m_treemap = m_treemap_builder(multi_tree, m_rect, timestep_weights(m_timestep));
	m_tmp = std::make_shared<TreemapPainting<Named>>(*m_treemap, *m_treemap);
	updated_treemap();
}

void TreemapDemo::create_info_box(Point<Inexact> pt, const NPN& node) {
	clear_info_box();
	if (m_treemap->nodeRegion(node) == std::nullopt) {
		return;
	}
	m_info_box = new QFrame(m_renderer);
	m_info_box_position = pt;
	auto q_pt = m_renderer->convertPoint(pt);
	m_info_box->move({static_cast<int>(q_pt.x()), static_cast<int>(q_pt.y())});
	m_info_box->resize(150, 180);
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
	auto nv = CGAL::to_double(timestep_weights(m_timestep)(node));
	weight_s.unsetf(std::ios::showpoint);
	if (nv < 1E9) {
		weight_s.precision(std::ceil(std::log10(nv)) + 1);
	} else {
		weight_s.precision(2);
	}
	weight_s << "Weight: " << std::noshowpoint << nv;
	auto* leaf_weight = new QLabel(QString::fromStdString(weight_s.str()));
	vLayout->addWidget(leaf_weight);

	auto* name_label = new QLabel(QString::fromStdString(node->value.name));
	vLayout->addWidget(name_label);

	auto* hLayout = new QHBoxLayout();
	auto* dec_weight = new QToolButton();
	dec_weight->setText("-");
	hLayout->addWidget(dec_weight);

//	auto rebuild = [node, pt, this]() mutable {
//		update_weights(node);
//		m_treemap = build_treemap(m_treemap->m_tree, weight_getter);
//		auto tmp = std::make_shared<TreemapPainting>(*m_treemap);
//		m_renderer->clear();
//		m_renderer->addPainting(tmp, "Treemap");
//		auto np = std::make_shared<NodePainting>(*m_treemap, m_selected_node);
//		m_renderer->addPainting(np, "Node highlight");
//		m_renderer->repaint();
//		create_info_box(pt, node);
//	};
//
//	connect(dec_weight, &QToolButton::clicked, [node, rebuild]() mutable {
//		if (node->value <= 1) return;
//		node->value -= 1;
//		rebuild();
//	});
//	auto* inc_weight = new QToolButton();
//	inc_weight->setText("+");
//	hLayout->addWidget(inc_weight);
//	connect(inc_weight, &QToolButton::clicked, [node, rebuild]() mutable {
//		node->value += 1;
//		rebuild();
//	});
//	vLayout->addLayout(hLayout);
	std::stringstream area_s;
	auto area = abs(CGAL::to_double(m_treemap->nodeRegion(node)->area()));
	if (area < 1E9) {
		area_s.precision(std::ceil(std::log10(area)) + 1);
	} else {
		area_s.precision(2);
	}
	area_s.unsetf(std::ios::showpoint);
	area_s << "Area: " << area;
	auto* region_area = new QLabel(QString::fromStdString(area_s.str()));
	vLayout->addWidget(region_area);
	std::stringstream aspect_ratio_s;
	auto aspect_ratio = aspect_ratio_square_percentage(*(m_treemap->nodeRegion(node)));
	aspect_ratio_s.precision(2);
	aspect_ratio_s.unsetf(std::ios::showpoint);
	aspect_ratio_s << "Aspect ratio: " << aspect_ratio;
	auto* aspect_ratio_l = new QLabel(QString::fromStdString(aspect_ratio_s.str()));
	vLayout->addWidget(aspect_ratio_l);
	auto* close_button = new QPushButton("Close");
	connect(close_button, &QPushButton::clicked, [this](){
		clear_info_box();
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

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	TreemapDemo demo;
	demo.show();
	QApplication::exec();
}