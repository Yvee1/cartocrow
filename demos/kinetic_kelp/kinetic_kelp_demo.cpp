#include "kinetic_kelp_demo.h"

#include <QApplication>
#include <QTimer>
#include <QElapsedTimer>
#include <QDockWidget>
#include <QToolBar>
#include <QHBoxLayout>
#include <QPushButton>
#include <QIcon>

#include "cartocrow/kinetic_kelp/parse_input.h"
#include "cartocrow/kinetic_kelp/moving_cat_point.h"

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

struct DrawSettings {
    std::vector<Color> colors;
    double strokeWidth = 2.0;
};

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

class KelpPainting : public GeometryPainting {
private:
    Color m_color;
    std::vector<Point<Inexact>> m_points;
    std::vector<Segment<Inexact>> m_links;
    double m_radius;

public:
    KelpPainting(Color color, std::vector<Point<Inexact>> points, std::vector<Segment<Inexact>> links, double radius) :
        m_color(color), m_points(std::move(points)), m_links(std::move(links)), m_radius(radius) {}

    Polygon<Inexact> segmentToRectangle(const Segment<Inexact>& s, const Number<Inexact>& w) const {
        auto p = s.source();
        auto q = s.target();
        Vector<Inexact> d = q - p;
        auto dl = sqrt(CGAL::to_double(d.squared_length()));
        Vector<Inexact> normalized = d / dl;
        auto perp = normalized.perpendicular(CGAL::COUNTERCLOCKWISE) * w / 2;

        Polygon<Inexact> poly;
        Point<Inexact> p1 = p - perp;
        Point<Inexact> p2 = q + normalized * w / 10 - perp;
        Point<Inexact> p3 = q + normalized * w / 10 + perp;
        Point<Inexact> p4 = p + perp;

        poly.push_back(p1);
        poly.push_back(p2);
        poly.push_back(p3);
        poly.push_back(p4);

        return poly;
    }

    void paint(GeometryRenderer &renderer) const override {
        renderer.setMode(GeometryRenderer::fill);
        renderer.setFill(m_color);
        for (const auto& l : m_links) {
            renderer.draw(segmentToRectangle(l, m_radius / 2));
        }
        for (const auto& p : m_points) {
            renderer.setFill(m_color);
            renderer.draw(Circle<Inexact>(p, m_radius * m_radius));
            renderer.setFill(Color(0, 0, 0));
            renderer.draw(Circle<Inexact>(p, m_radius * m_radius / 50));
        }
    }
};

KineticKelpDemo::KineticKelpDemo() {
    setWindowTitle("KineticKelp");
    m_renderer = new GeometryWidget();
    m_renderer->setDrawAxes(false);
    setCentralWidget(m_renderer);

    std::string filePath = "data/kinetic_kelp/trajectory.ipe";
    m_input = Input(parseMovingPoints(filePath));

    m_timeControl = new TimeControlToolBar(m_renderer, m_input.timespan().second);

    std::vector colors({CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange});
    DrawSettings ds;
    ds.colors = colors;

    m_renderer->fitInView(bounds(m_input.movingCatPoints()));

    computeMSTs(0);

    m_renderer->addPainting([this](GeometryRenderer& renderer) {
        computeMSTs(m_timeControl->time());

        renderer.setMode(GeometryRenderer::stroke);
        renderer.setStroke(Color(0, 0, 0), m_pointRadius / 3.5, true);

        for (int k = 0; k < m_input.numCategories(); ++k) {
            auto& mst = m_msts[k];
            auto& dt = m_dts[k];
            for (const TGEdge &ed: mst) {
                TGVertex svd = CGAL::source(ed, dt);
                TGVertex tvd = CGAL::target(ed, dt);
                DT::Vertex_handle sv = svd;
                DT::Vertex_handle tv = tvd;
                renderer.draw(Segment<Inexact>(sv->point(), tv->point()));
            }
        }
    }, "MSTs");

//    m_renderer->addPainting([ds, this](GeometryRenderer& renderer) {
//        for (const auto& mcp : m_input.movingCatPoints()) {
//            drawTrajectory(renderer, mcp, ds);
//        }
//    }, "CatPoint Trajectories");

    m_renderer->addPainting([ds, this](GeometryRenderer& renderer) {
        for (const auto& mcp : m_input.movingCatPoints()) {
            drawMovingCatPoint(renderer, mcp, m_timeControl->time(), ds);
        }
    }, "Moving points");

    for (int k = 0; k < m_input.numCategories(); ++k) {
        auto& mst = m_msts[k];
        auto& dt = m_dts[k];

        std::vector<Segment<Inexact>> links;
        for (const TGEdge &ed: mst) {
            TGVertex svd = CGAL::source(ed, dt);
            TGVertex tvd = CGAL::target(ed, dt);
            DT::Vertex_handle sv = svd;
            DT::Vertex_handle tv = tvd;
            links.emplace_back(sv->point(), tv->point());
        }

        std::vector<Point<Inexact>> points;
        std::copy(dt.points_begin(), dt.points_end(), std::back_inserter(points));

        auto kelpPainting = std::make_shared<KelpPainting>(colors[k], std::move(points), std::move(links), 4 * m_pointRadius);
    }
}

void KineticKelpDemo::fitToScreen() {
    Box box = bounds(m_input.movingCatPoints());
    auto delta = CGAL::to_double(8 * m_pointRadius);
    Box expanded(box.xmin() - delta, box.ymin() - delta, box.xmax() + delta, box.ymax() + delta);
    m_renderer->fitInView(expanded);
}

void KineticKelpDemo::resizeEvent(QResizeEvent *event) {
    fitToScreen();
    m_timeControl->resized();
}

void KineticKelpDemo::computeMSTs(double time) {
    m_dts.clear();
    m_msts.clear();

    for (int k = 0; k < m_input.numCategories(); ++k) {
        DT& dt = m_dts.emplace_back();

        for (const auto &[c, t]: m_input.movingCatPoints()) {
            if (c == k) {
                dt.insert(t.posAtTime(time));
            }
        }

        // Associate indices to the vertices
        VertexIndexMap vertex_id_map;
        VertexIdPropertyMap vertex_index_pmap(vertex_id_map);
        int index = 0;

        for (TGVertex vd: vertices(dt))
            vertex_id_map[vd] = index++;

        std::list<TGEdge>& mst = m_msts.emplace_back();
        boost::kruskal_minimum_spanning_tree(dt, std::back_inserter(mst),
                                             vertex_index_map(vertex_index_pmap));
    }
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    KineticKelpDemo demo;
    demo.show();
    app.exec();
}
