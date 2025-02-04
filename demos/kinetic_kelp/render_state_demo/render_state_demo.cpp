#include "render_state_demo.h"

#include <QApplication>

#include "cartocrow/renderer/geometry_widget.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"
#include "cartocrow/kinetic_kelp/kinetic_kelp_painting.h"

#include "../colors/colors.h"

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::kinetic_kelp;

RenderStateDemo::RenderStateDemo() {
    setWindowTitle("KineticKelp: Render state");
    auto renderer = new GeometryWidget();
    renderer->setDrawAxes(false);
    setCentralWidget(renderer);

    auto input = std::make_shared<InputInstance>(std::vector<CatPoint>({
        {0, {-100, 0}},
        {0, {0, 0}},
        {0, {-50, 71}},
        {1, {-25, -10}},
        {1, {-75, 0}},
        {2, {-125, 0}},
        {2, {25, 0}}
    }));

    Settings settings;
    settings.pointRadius = 10.0;
    settings.edgeWidth = 5.0;

    State state;
    state.msts.push_back({{0, 1}, {1, 2}});
    state.msts.push_back({{3, 4}});
    state.msts.push_back({{5, 6}});
    state.mstEdgeTopology[{0, 1}] = EdgeTopology(0, 1, {
        {4, 10.0, 15.0, CGAL::CLOCKWISE},
        {3, 10.0, 15.0, CGAL::COUNTERCLOCKWISE},
    });
    state.mstEdgeTopology[{1, 2}] = EdgeTopology(1, 2, {});
    state.mstEdgeTopology[{3, 4}] = EdgeTopology(3, 4, {});
    state.mstEdgeTopology[{5, 6}] = EdgeTopology(5, 6, {
        {0, 10.0, 15.0, CGAL::CLOCKWISE},
        {4, 15.0, 20.0, CGAL::CLOCKWISE},
        {1, 10.0, 15.0, CGAL::CLOCKWISE},
    });

    auto stateGeometry = std::make_shared<StateGeometry>(stateToGeometry(state, *input, settings));
    auto stateGeometryP = std::make_shared<StateGeometryPainting>(stateGeometry);
    renderer->addPainting(stateGeometryP, "State geometry");

    KineticKelpPainting::DrawSettings ds;
    ds.colors = {CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange};
    ds.markRadius = 1.0;
    ds.strokeWidth = 1.0;
    ds.smoothing = 5.0;
    auto kkPainting = std::make_shared<KineticKelpPainting>(stateGeometry, input, ds);
    renderer->addPainting(kkPainting, "KineticKelp");
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    RenderStateDemo demo;
    demo.show();
    app.exec();
}
