#include "kinetic_kelp_demo.h"

#include <QApplication>
#include <QTimer>
#include <QElapsedTimer>
#include <QComboBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QLabel>

#include "cartocrow/core/transform_helpers.h"
#include "cartocrow/renderer/svg_renderer.h"

#include "cartocrow/kinetic_kelp/parse_input.h"
#include "cartocrow/kinetic_kelp/moving_cat_point.h"
#include "cartocrow/kinetic_kelp/route_edges.h"
#include "cartocrow/kinetic_kelp/pseudotriangulation_painting.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"
#include "cartocrow/kinetic_kelp/point_id_painting.h"

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

	auto* timeMultiplierLabel = new QLabel("CSV time multiplier");
	m_timeMultiplierSpinBox = new QDoubleSpinBox();
	m_timeMultiplierSpinBox->setMinimum(0);
	m_timeMultiplierSpinBox->setMaximum(10);
	// reasonable values:
	// 0.00000001 for carnivores
	// 0.0001 for PUBG
	m_timeMultiplierSpinBox->setDecimals(10);
	m_timeMultiplierSpinBox->setValue(0.00000001);
	vLayout->addWidget(timeMultiplierLabel);
	vLayout->addWidget(m_timeMultiplierSpinBox);

    m_filePath = "data/kinetic_kelp/disjoint-fails.ipe";
    m_input = Input(parseIpeAsMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));

    m_timeControl = new TimeControlToolBar(m_renderer, m_input.timespan().first, m_input.timespan().second, std::round(1000.0 / fps->value()));

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

    auto* kineticKelpSettings = new QLabel("<h3>KineticKelp settings</h3>");
    vLayout->addWidget(kineticKelpSettings);

    auto* justPoints = new QCheckBox("Only points");
    vLayout->addWidget(justPoints);

    auto* kelpRadiusLabel = new QLabel("Kelp radius");
    m_kelpRadius = new QSlider();
    vLayout->addWidget(kelpRadiusLabel);
    vLayout->addWidget(m_kelpRadius);
    m_kelpRadius->setMinimum(1);
    m_kelpRadius->setMaximum(1000);
    m_kelpRadius->setValue(100);
    m_kelpRadius->setOrientation(Qt::Horizontal);

    m_smoothCheckBox = new QCheckBox("Smooth");
    m_smoothCheckBox->setChecked(true);
    vLayout->addWidget(m_smoothCheckBox);

    m_insertDelete = new QCheckBox("Insert/delete?");
    vLayout->addWidget(m_insertDelete);

    auto* colorsSelector = new QComboBox();
    colorsSelector->addItem("ColorBrewer");
    colorsSelector->addItem("Tableau 20; light then dark");
    colorsSelector->setCurrentIndex(0);
    vLayout->addWidget(colorsSelector);

    connect(m_interpolationTimeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), [this]() {
        m_input = Input(parseIpeAsMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));
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
        if (m_filePath.extension() == ".ipe") {
            m_input = Input(parseIpeAsMovingPoints(m_filePath, m_interpolationTimeSpinBox->value()));
        } else if (m_filePath.extension() == ".csv") {
			auto mcps = parseCSVAsMovingPoints(m_filePath, m_timeMultiplierSpinBox->value());
            auto inputBounds = bounds(mcps);
            Rectangle<Inexact> outputBounds(0.0, 0.0, 1000.0, 1000.0);
            auto trans = fitInto(inputBounds, outputBounds);
            std::vector<MovingCatPoint> transformed;
            for (const auto& mcp : mcps) {
                transformed.push_back(mcp.transform(trans));
            }
            m_input = Input(transformed);
			Vector<Inexact> testing = Vector<Inexact>(1, 0).transform(trans);
			std::cout << "Scaled by: " << sqrt(testing.squared_length()) << std::endl;
        }
        m_timeControl->setStartTime(m_input.timespan().first);
        m_timeControl->setEndTime(m_input.timespan().second);
        m_timeControl->restart();
        std::cout << "Time span: " << m_input.timespan().first << " -- " << m_input.timespan().second << std::endl;
        initialize();
    });

    connect(recomputeCheckBox, &QCheckBox::stateChanged, [this, recomputeCheckBox]() {
        m_recompute = recomputeCheckBox->isChecked();
    });

    m_drawSettings = std::make_shared<DrawSettings>();
	m_drawSettings->colors = CB::lights;
//    m_drawSettings.colors = tableau::firstLightThenDark;
    m_renderer->fitInView(bounds(m_input.movingCatPoints()));

    initialize();

	connect(m_timeControl, &TimeControlToolBar::ticked, [saveToSvg, pauseOnEventCheckBox, fixButton, this](int tick, double time) {
        std::cout << "[update] tick: " << tick << " time: " << time << std::endl;
        if (m_recompute) {
            initialize();
        } else {
			if (!pauseOnEventCheckBox->isChecked()) {
				update(time);
			} else {
				*m_brokenState = *m_state;
				auto newPt = Pseudotriangulation(*m_pt, *m_state, *m_brokenState);
				*m_brokenPt = std::move(newPt);
				auto result = updateDebug(time);
				if (result.has_value()) {
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
			m_brokenPt->fix(**m_failedCertificate, *m_brokenState, m_settings);
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

    connect(m_timeControl, &TimeControlToolBar::restarted, [this, fixButton]() {
        initialize();
	  	m_timeControl->setPlayPauseEnabled(true);
		fixButton->setEnabled(false);
    });

    connect(justPoints, &QCheckBox::stateChanged, [this, justPoints]() {
        m_justPoints = justPoints->isChecked();
        initialize();
    });

    connect(m_kelpRadius, &QSlider::valueChanged, [this]() {
        initialize();
    });

    connect(m_smoothCheckBox, &QCheckBox::stateChanged, [this]() {
        m_kelps->clear();
        try {
            stateGeometryToKelps(*m_stateGeometry, *m_inputInstance,
                                 m_smoothCheckBox->isChecked() ? std::optional(m_drawSettings->smoothing) :
                                 std::nullopt, std::back_inserter(*m_kelps));
        } catch(const std::exception& exception) {
            std::cerr << exception.what() << std::endl;
        }
        repaint();
    });

    connect(m_insertDelete, &QCheckBox::stateChanged, [this]() {
        initialize();
        repaint();
    });

    connect(colorsSelector, &QComboBox::currentTextChanged, [this, colorsSelector]() {
        switch (colorsSelector->currentIndex()) {
            case 0: {
                m_drawSettings->colors = CB::lights;
                break;
            };
            case 1: {
                m_drawSettings->colors = tableau::firstLightThenDark;
                break;
            };
            default: {
                std::cerr << "Unknown colors: " << colorsSelector->currentText().toStdString() << std::endl;
                break;
            }
        }
        repaint();
    });
}

void KineticKelpDemo::initialize() {
    m_renderer->clear();

    m_settings.kelpRadius = m_kelpRadius->value() / 10.0;
    m_settings.edgeWidth = m_settings.kelpRadius / 2;

    m_drawSettings->markRadius = CGAL::to_double(m_settings.kelpRadius / 10);
    m_drawSettings->strokeWidth = CGAL::to_double(m_settings.kelpRadius / 10);
    m_drawSettings->smoothing = CGAL::to_double(m_settings.kelpRadius / 2);

    m_inputInstance = std::make_shared<InputInstance>(m_input.instance(m_timeControl->time(), !m_insertDelete->isChecked()));

    if (!m_justPoints) {
        auto pr = std::make_shared<PaintingRenderer>();
        auto [stateTopology, stateGeometry] = routeEdges(*m_inputInstance, m_settings, *pr);
        m_stateGeometry = std::move(stateGeometry);
        m_state = stateTopology;
    } else {
        m_state = std::make_shared<State>();
        m_stateGeometry = std::make_shared<StateGeometry>(*m_state, *m_inputInstance, m_settings);
    }
	m_brokenState = std::make_shared<State>(*m_state);

    auto stateGeometryP = std::make_shared<StateGeometryPainting>(m_stateGeometry);
    m_renderer->addPainting(stateGeometryP, "State geometry");

    if (!m_justPoints) {
        auto pr1 = std::make_shared<PaintingRenderer>();
        auto [pt, ptg] = PseudotriangulationGeometry::pseudotriangulationTangents(*m_state, *m_stateGeometry);
        m_pt = std::make_shared<Pseudotriangulation>(pt);
        m_brokenPt = std::make_shared<Pseudotriangulation>(pt);
        m_ptg = std::make_shared<PseudotriangulationGeometry>(ptg);
        auto ptPainting = std::make_shared<PseudotriangulationPainting>(m_ptg);
        m_renderer->addPainting(ptPainting, "Pseudotriangulation");
        auto ptCPainting = std::make_shared<PseudotriangulationCertificatesPainting>(m_pt, m_ptg, m_state,
                                                                                     m_inputInstance, m_settings);
        m_renderer->addPainting(ptCPainting, "Certificates");
	}

	auto pointIdPainting = std::make_shared<PointIdPainting>(m_inputInstance);
	m_renderer->addPainting(pointIdPainting, "Point IDs");

	if (!m_justPoints) {
        m_failurePainting = std::make_shared<PaintingRenderer>();
        m_renderer->addPainting(m_failurePainting, "Certificate failure");
    }

    m_kelps = std::make_shared<std::vector<Kelp>>();
    try {
        stateGeometryToKelps(*m_stateGeometry, *m_inputInstance, m_smoothCheckBox->isChecked() ? std::optional(m_drawSettings->smoothing) :
                                                                 std::nullopt, std::back_inserter(*m_kelps));
    } catch (...) {}
    m_kkPainting = std::make_shared<KineticKelpPainting>(m_kelps, m_inputInstance, m_drawSettings);
    m_renderer->addPainting(m_kkPainting, "KineticKelp");
}

bool KineticKelpDemo::update(double time) {
	bool noIssues = true;

    auto newInputInstance = m_input.instance(time, !m_insertDelete->isChecked());
	auto newStateGeometry = StateGeometry(*m_state, newInputInstance, m_settings);
    if (!m_justPoints) {
        auto newPtg = PseudotriangulationGeometry(*m_pt, *m_state, newStateGeometry, newInputInstance);

        bool allGood = false;
        while (!allGood) {
            bool foundInvalidCertificate = false;
            for (auto &c: m_pt->m_certificates) {
                if (!m_pt->valid(c, *m_state, newPtg, newInputInstance)) {
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

        *m_ptg = std::move(newPtg);
    }

    *m_inputInstance = std::move(newInputInstance);
    *m_stateGeometry = std::move(newStateGeometry);

    m_kelps->clear();
    try {
        stateGeometryToKelps(*m_stateGeometry, *m_inputInstance,m_smoothCheckBox->isChecked() ? std::optional(m_drawSettings->smoothing) :
                                                                std::nullopt, std::back_inserter(*m_kelps));
    } catch(const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
    }

	return noIssues;
}

std::optional<std::pair<std::shared_ptr<Pseudotriangulation::Certificate>, CertificateFailurePainting>> KineticKelpDemo::updateDebug(double time) {
	auto newInputInstance = m_input.instance(time, !m_insertDelete->isChecked());
	auto newStateGeometry = std::make_shared<StateGeometry>(*m_brokenState, newInputInstance, m_settings);
	auto newPtg = PseudotriangulationGeometry(*m_brokenPt, *m_brokenState, *newStateGeometry, newInputInstance);

	for (auto& c : m_brokenPt->m_certificates) {
		if (!m_brokenPt->valid(c, *m_brokenState, newPtg, newInputInstance)) {
            auto cSP = std::make_shared<Pseudotriangulation::Certificate>(c);
			CertificateFailurePainting cfp(cSP, *m_brokenPt, std::move(newPtg), *m_brokenState, newStateGeometry, newInputInstance, m_settings);
			return std::pair(cSP, cfp);
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
        stateGeometryToKelps(*m_stateGeometry, *m_inputInstance, m_smoothCheckBox->isChecked() ? std::optional(m_drawSettings->smoothing) :
                                                                std::nullopt, std::back_inserter(*m_kelps));
	} catch(const std::exception& exception) {
		std::cerr << exception.what() << std::endl;
	}

	return std::nullopt;
}

void KineticKelpDemo::fitToScreen() {
    Box box = bounds(m_input.movingCatPoints());
    auto delta = CGAL::to_double(m_settings.kelpRadius + 2 * m_settings.edgeWidth);
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
