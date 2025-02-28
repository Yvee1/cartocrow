#include "kinetic_kelp_demo.h"

#include <QApplication>
#include <QTimer>
#include <QElapsedTimer>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QFileDialog>
#include <QLabel>

#include "cartocrow/renderer/svg_renderer.h"

#include "cartocrow/kinetic_kelp/parse_input.h"
#include "cartocrow/kinetic_kelp/moving_cat_point.h"
#include "cartocrow/kinetic_kelp/route_edges.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation_painting.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"

#include "cartocrow/renderer/painting_renderer.h"
#include "colors/colors.h"

#include <fstream>

using namespace cartocrow::renderer;

Box bounds(const std::vector<CatPoint>& points) {
    std::vector<Point<Inexact>> pts;
    for (const auto& [_, p] : points) {
        pts.push_back(approximate(p));
    }
    return CGAL::bbox_2(pts.begin(), pts.end());
}

Box bounds(const std::vector<MovingCatPoint>& movingPoints) {
    std::vector<Point<Inexact>> pts;
    for (const auto& [_, t] : movingPoints) {
        for (const auto& p : t.m_points) {
            pts.push_back(p.point);
        }
    }
    return CGAL::bbox_2(pts.begin(), pts.end());
}

void drawTrajectory(GeometryRenderer& renderer, const MovingCatPoint& mcp, const DrawSettings& ds) {
    renderer.setStroke(ds.colors[mcp.category], ds.strokeWidth);
    renderer.draw(mcp.trajectory.polyline());
    for (const auto& [_, p] : mcp.trajectory.m_points) {
        renderer.draw(p);
    }
}

void drawMovingCatPoint(GeometryRenderer& renderer, const MovingCatPoint& mcp, double time, const DrawSettings& ds) {
    renderer.setStroke(ds.colors[mcp.category], ds.strokeWidth);
    renderer.draw(mcp.trajectory.posAtTime(time));
}

KineticKelpDemo::KineticKelpDemo() {
	bool saveToSvg = false;

    setWindowTitle("KineticKelp");
    m_renderer = new GeometryWidget();
    m_renderer->setDrawAxes(false);
    setCentralWidget(m_renderer);

    auto* dockWidget = new QDockWidget();
    addDockWidget(Qt::RightDockWidgetArea, dockWidget);
    auto* vWidget = new QWidget();
    auto* vLayout = new QVBoxLayout(vWidget);
    vLayout->setAlignment(Qt::AlignTop);
    dockWidget->setWidget(vWidget);

	auto* timeSettingsLabel = new QLabel("<h3>Time settings</h3>");
	auto* fpsLabel = new QLabel("FPS");
	auto* fps = new QSpinBox();
	fps->setMinimum(1);
	fps->setMaximum(144);
	fps->setValue(60);
	vLayout->addWidget(timeSettingsLabel);
	vLayout->addWidget(fpsLabel);
	vLayout->addWidget(fps);

	auto* inputSettingsLabel = new QLabel("<h3>Input settings</h3>");
    auto* interpolationTimeLabel = new QLabel("Seconds between polyline vertices");
    m_interpolationTimeSpinBox = new QSpinBox();
    m_interpolationTimeSpinBox->setMinimum(1);
    m_interpolationTimeSpinBox->setMaximum(15);
    m_interpolationTimeSpinBox->setValue(5);
	vLayout->addWidget(inputSettingsLabel);
    vLayout->addWidget(interpolationTimeLabel);
    vLayout->addWidget(m_interpolationTimeSpinBox);

    m_filePath = "data/kinetic_kelp/easy.ipe";
    m_input = Input(parseMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));

    m_timeControl = new TimeControlToolBar(m_renderer, m_input.timespan().second, std::round(1000.0 / fps->value()));

    auto* loadFileButton = new QPushButton("Load file");
    vLayout->addWidget(loadFileButton);

	auto* simulationSettingsLabel = new QLabel("<h3>Simulation settings</h3>");
	vLayout->addWidget(simulationSettingsLabel);
    auto* recomputeCheckBox = new QCheckBox("Recompute");
    vLayout->addWidget(recomputeCheckBox);

    auto* pauseOnEventCheckBox = new QCheckBox("Pause on event");
    vLayout->addWidget(pauseOnEventCheckBox);

	auto* fixButton = new QPushButton("Fix certificate");
	vLayout->addWidget(fixButton);

	fixButton->setEnabled(false);

    connect(m_interpolationTimeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), [this]() {
        m_input = Input(parseMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));
        m_timeControl->restart();
        m_timeControl->setEndTime(m_input.timespan().second);
        initialize();
    });

	connect(fps, QOverload<int>::of(&QSpinBox::valueChanged), [this, fps]() {
		m_timeControl->setInterval(std::round(1000.0 / fps->value()));
	});

    connect(loadFileButton, &QPushButton::clicked, [this]() {
        QString startDir = "data/kinetic_kelp";
        std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select trajectory file"), startDir).toStdString();
        if (filePath == "") return;
        m_filePath = filePath;
        m_input = Input(parseMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));
        m_timeControl->restart();
        m_timeControl->setEndTime(m_input.timespan().second);
        initialize();
    });

    connect(recomputeCheckBox, &QCheckBox::stateChanged, [this, recomputeCheckBox]() {
        m_recompute = recomputeCheckBox->isChecked();
    });

	m_drawSettings.colors = {CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange};
	m_drawSettings.markRadius = 1.0;
	m_drawSettings.strokeWidth = 1.0;
	m_drawSettings.smoothing = 5.0;

    m_renderer->fitInView(bounds(m_input.movingCatPoints()));

    initialize();

	connect(m_timeControl, &TimeControlToolBar::ticked, [saveToSvg, pauseOnEventCheckBox, fixButton, this](int tick, double time) {
        if (m_recompute) {
            initialize();
        } else {
			if (!pauseOnEventCheckBox->isChecked()) {
				update(time);
			} else {
				std::cout << "Doing stuff" << std::endl;
				for (const auto& obj : m_pt->m_tangentObjects) {
					if (obj->straightId.has_value()) {
						auto& [edge, it] = *obj->straightId;
						std::cout << std::distance(m_state->edgeTopology[edge].orbits.cbegin(), it) << std::endl;
					}
				}
				*m_brokenState = *m_state;
				std::cout << "Wtf" << std::endl;
				for (const auto& obj : m_pt->m_tangentObjects) {
					if (obj->straightId.has_value()) {
						auto& [edge, it] = *obj->straightId;
						std::cout << std::distance(m_state->edgeTopology[edge].orbits.cbegin(), it) << std::endl;
					}
				}
				auto newPt = Pseudotriangulation(*m_pt, *m_state, *m_brokenState);
				*m_brokenPt = std::move(newPt);
				auto result = updateDebug(time);
				if (result.has_value()) {
					std::cout << "[update] Time: " << time << std::endl;
					m_timeControl->playOrPause();
					auto [c, painting] = *result;
					m_failedCertificate = c;
					fixButton->setEnabled(true);
					m_failurePainting->clear();
					painting.paint(*m_failurePainting);
					m_timeControl->setPlayPauseEnabled(false);
				}
			}
        }
	    repaint();

        if (saveToSvg) {
            SvgRenderer svgRenderer;
            svgRenderer.addPainting(
                    [this](GeometryRenderer& renderer) {
                        m_kkPainting->paint(renderer);
                    },
                    "KineticKelp");
            std::stringstream filename;
            filename << "frames/frame-" << std::setfill('0') << std::setw(5) << tick;
            svgRenderer.save(filename.str() + ".svg");
        }
	});

	connect(fixButton, &QPushButton::clicked, [this, pauseOnEventCheckBox, fixButton]() {
		if (m_failedCertificate.has_value()) {
			auto time = m_timeControl->time();
			std::cout << "[fixButton] Time: " << time << std::endl;
			m_brokenPt->fix(*m_failedCertificate, *m_brokenState, m_settings);
			m_failedCertificate = std::nullopt;
			m_timeControl->setPlayPauseEnabled(true);
			m_failurePainting->clear();
			fixButton->setEnabled(false);
			if (!pauseOnEventCheckBox->isChecked()) {
				update(time);
			} else {
				auto result = updateDebug(time);
				if (result.has_value()) {
					auto [c, painting] = *result;
					m_failedCertificate = c;
					fixButton->setEnabled(true);
					painting.paint(*m_failurePainting);
					m_timeControl->setPlayPauseEnabled(false);
				}
			}
			repaint();
		}
	});

    connect(m_timeControl, &TimeControlToolBar::restarted, [this]() {
        initialize();
	  	m_timeControl->setPlayPauseEnabled(true);
    });
}

void KineticKelpDemo::initialize() {
    m_renderer->clear();

    m_settings.vertexRadius = 10.0;
    m_settings.edgeWidth = 5.0;

    m_inputInstance = std::make_shared<InputInstance>(m_input.instance(m_timeControl->time()));
    auto pr = std::make_shared<PaintingRenderer>();
    auto [stateTopology, stateGeometry] = routeEdges(*m_inputInstance, m_settings, *pr);
    m_stateGeometry = std::move(stateGeometry);
    m_state = stateTopology;
	m_brokenState = stateTopology;

    auto stateGeometryP = std::make_shared<StateGeometryPainting>(m_stateGeometry);
    m_renderer->addPainting(stateGeometryP, "State geometry");

    auto pr1 = std::make_shared<PaintingRenderer>();
    auto [pt, ptg] = PseudotriangulationGeometry::pseudotriangulationTangents(*m_state, *m_stateGeometry);
    m_pt = std::make_shared<Pseudotriangulation>(pt);
	m_brokenPt = std::make_shared<Pseudotriangulation>(pt);
    m_ptg = std::make_shared<PseudotriangulationGeometry>(ptg);
    auto ptPainting = std::make_shared<PseudotriangulationPainting>(m_ptg);
    m_renderer->addPainting(ptPainting, "Pseudotriangulation");
	auto ptCPainting = std::make_shared<PseudotriangulationCertificatesPainting>(m_pt, m_ptg, m_state, m_inputInstance, m_settings);
	m_renderer->addPainting(ptCPainting, "Certificates");

	m_failurePainting = std::make_shared<PaintingRenderer>();
	m_renderer->addPainting(m_failurePainting, "Certificate failure");

    m_kelps = std::make_shared<std::vector<Kelp>>();
    try {
        stateGeometrytoKelps(*m_stateGeometry, *m_inputInstance, m_drawSettings.smoothing, std::back_inserter(*m_kelps));
    } catch (...) {}
    m_kkPainting = std::make_shared<KineticKelpPainting>(m_kelps, m_inputInstance, m_drawSettings);
    m_renderer->addPainting(m_kkPainting, "KineticKelp");
}

bool KineticKelpDemo::update(double time) {
	bool noIssues = true;

    auto newInputInstance = m_input.instance(time);
	auto newStateGeometry = StateGeometry(*m_state, newInputInstance, m_settings);
	auto newPtg = PseudotriangulationGeometry(*m_pt, *m_state, newStateGeometry, newInputInstance);

	bool allGood = false;
	while (!allGood) {
		bool foundInvalidCertificate = false;
		for (auto& c : m_pt->m_tangentEndpointCertificates) {
			if (!c.valid(*m_pt, *m_state, newPtg)) {
				m_pt->fix(c, *m_state, m_settings);
				newStateGeometry = StateGeometry(*m_state, newInputInstance, m_settings);
				newPtg = PseudotriangulationGeometry(*m_pt, *m_state, newStateGeometry, newInputInstance);
				foundInvalidCertificate = true;
				noIssues = false;
				break;
			}
		}
		if (!foundInvalidCertificate)
			allGood = true;
	}

	*m_inputInstance = std::move(newInputInstance);
	*m_stateGeometry = std::move(newStateGeometry);
	*m_ptg = std::move(newPtg);

    m_kelps->clear();
    try {
        stateGeometrytoKelps(*m_stateGeometry, *m_inputInstance, m_drawSettings.smoothing, std::back_inserter(*m_kelps));
    } catch(const std::runtime_error& error) {
        std::cerr << error.what() << std::endl;
    }

	return noIssues;
}

std::optional<std::pair<Pseudotriangulation::TangentEndpointCertificate, CertificateFailurePainting>> KineticKelpDemo::updateDebug(double time) {
	auto newInputInstance = m_input.instance(time);
	auto newStateGeometry = std::make_shared<StateGeometry>(*m_brokenState, newInputInstance, m_settings);
	auto newPtg = PseudotriangulationGeometry(*m_brokenPt, *m_brokenState, *newStateGeometry, newInputInstance);

	for (auto& c : m_brokenPt->m_tangentEndpointCertificates) {
		if (!c.valid(*m_brokenPt, *m_brokenState, newPtg)) {
			CertificateFailurePainting cfp(c, *m_brokenPt, std::move(newPtg), *m_brokenState, newStateGeometry, newInputInstance, m_settings);
			return std::pair(c, cfp);
		}
	}

	*m_state = *m_brokenState;
	auto copiedPt = Pseudotriangulation(*m_brokenPt, *m_brokenState, *m_state);
	*m_pt = std::move(copiedPt);
	*m_inputInstance = std::move(newInputInstance);
	*m_stateGeometry = std::move(*newStateGeometry);
	auto copiedPtg = PseudotriangulationGeometry(*m_pt, *m_state, *m_stateGeometry, *m_inputInstance); // can be made more efficient by implementing a copy constructor
	*m_ptg = std::move(copiedPtg);
	m_kelps->clear();
	try {
		stateGeometrytoKelps(*m_stateGeometry, *m_inputInstance, m_drawSettings.smoothing, std::back_inserter(*m_kelps));
	} catch(const std::runtime_error& error) {
		std::cerr << error.what() << std::endl;
	}

	return std::nullopt;
}

void KineticKelpDemo::fitToScreen() {
    Box box = bounds(m_input.movingCatPoints());
    auto delta = CGAL::to_double(m_settings.vertexRadius + 2 * m_settings.edgeWidth);
    Box expanded(box.xmin() - delta, box.ymin() - delta, box.xmax() + delta, box.ymax() + delta);
    m_renderer->fitInView(expanded);
}

void KineticKelpDemo::resizeEvent(QResizeEvent *event) {
    fitToScreen();
    m_timeControl->resized();
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    KineticKelpDemo demo;
    demo.show();
    app.exec();
}
