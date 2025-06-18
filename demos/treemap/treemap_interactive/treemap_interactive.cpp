#include "treemap_interactive.h"
#include "cartocrow/treemap/aspect_ratio.h"
#include "cartocrow/treemap/parse_tree.h"
#include "cartocrow/treemap/parse_csv_to_tree.h"
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>
#include <QComboBox>
#include "cartocrow/treemap/orthoconvex.h"
#include "cartocrow/treemap/convex.h"

#include <CGAL/Arr_accessor.h>


// Determine maximal segments.
// Make Editable that can:
// - move maximal segments (via modify_edge_ex and modify_vertex_ex of Arr_accessor)
// - collapse edges that define a non-standard split? L-shape/S-shape edges...
// - ¨pull out¨ edges to create an L-shape/S-shape

TreemapInteractive::TreemapInteractive() {
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

	m_treemap_builder = [](NP<Named>& tree, const Rectangle<K>& rect, NodeWeight<Named> w) {
		return std::make_shared<Treemap<Named>>(orthoconvex_treemap(tree, rect, w));
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
		load_file("data/wb-SM.POP.NETM-net-migration.data");

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
		start_dir = "data/";
		std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select treemap data file"), start_dir).toStdString();

		if (filePath == "") return;
		load_file(filePath);
		fileSelector->setText(QString::fromStdString(filePath.filename()));
	});
}

void TreemapInteractive::clear_info_box() {
	if (m_info_box != nullptr) {
		m_info_box->close();
	}
	m_info_box = nullptr;
	m_info_box_position = std::nullopt;
}

void TreemapInteractive::updated_treemap() {
	m_renderer->clear();
	m_tmp = std::make_shared<TreemapPainting<Named>>(*m_treemap, m_tmp->m_initial_treemap);
	m_renderer->addPainting(m_tmp, "Treemap");
	auto np = std::make_shared<NodePainting<Named>>(*m_treemap, m_selected_node);
	m_renderer->addPainting(np, "Node highlight");
	m_renderer->repaint();
	if (m_info_box != nullptr) {
		create_info_box(*m_info_box_position, *m_selected_node);
	}
	m_renderer->m_editables.push_back(std::make_unique<TreemapEditable>(m_renderer, m_treemap));
}

void TreemapInteractive::load_file(const std::filesystem::path& filePath) {
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

void TreemapInteractive::create_info_box(Point<Inexact> pt, const NPN& node) {
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

Arrangement<K>::Face_handle TreemapInteractive::face_at_point(const Point<K>& point) {
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

void TreemapInteractive::resizeEvent(QResizeEvent *event) {
	if (m_info_box == nullptr) return;
	auto qp = m_renderer->convertPoint(*m_info_box_position);
	m_info_box->move({static_cast<int>(qp.x()), static_cast<int>(qp.y())});
}

TreemapEditable::TreemapEditable(GeometryWidget* widget, std::shared_ptr<Treemap<Named>> treemap) :
      GeometryWidget::Editable(widget), m_treemap(std::move(treemap)) {}

std::optional<Treemap<Named>::MaximalSegmentId> TreemapEditable::closestMaximalSegment(Point<Inexact> location, double radius) const {
	double minDist = std::numeric_limits<double>::infinity();
	std::optional<Treemap<Named>::MaximalSegmentId> closestSegment;

	for (int i = 0; i < m_treemap->m_maximalSegments.size(); ++i) {
		auto& mSeg = m_treemap->m_maximalSegments[i];
		auto dist = CGAL::squared_distance(mSeg.segment, location);
		if (dist < minDist) {
			minDist = dist;
			closestSegment = i;
		}
	}

	if (closestSegment.has_value() && minDist < radius * radius) {
		return *closestSegment;
	} else {
		return std::nullopt;
	}
}

bool TreemapEditable::drawHoverHint(Point<cartocrow::Inexact> location, double radius) const {
	auto closest = closestMaximalSegment(location, radius);
	if (closest.has_value()) {
		m_widget->setStroke(Color(0, 0, 255), 3.0);
		m_widget->GeometryRenderer::draw(m_treemap->m_maximalSegments[*closest].segment);
		return true;
	}
	return false;
}

bool TreemapEditable::startDrag(Point<cartocrow::Inexact> location, double radius) {
	auto closest = closestMaximalSegment(location, radius);
	if (closest.has_value()) {
		m_dragging = closest;
		auto seg = m_treemap->m_maximalSegments[*m_dragging].segment;
		bool vertical = abs(seg.source().x() - seg.target().x()) < M_EPSILON;
		QApplication::setOverrideCursor(QCursor(vertical ? Qt::SizeHorCursor : Qt::SizeVerCursor));
		return true;
	}
	return false;
}

void TreemapEditable::endDrag() {
	QApplication::restoreOverrideCursor();
	m_dragging = std::nullopt;
}

void TreemapEditable::handleDrag(Point<cartocrow::Inexact> to) {
	if (!m_dragging.has_value()) return;
	auto seg = m_treemap->m_maximalSegments[*m_dragging].segment;
	bool vertical = abs(seg.source().x() - seg.target().x()) < M_EPSILON;
	m_treemap->moveMaximalSegment(*m_dragging, vertical ? to.x() - seg.source().x() : to.y() - seg.source().y());
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	TreemapInteractive demo;
	demo.show();
	QApplication::exec();
}