#include "render_state_demo.h"

#include <QApplication>

#include "cartocrow/renderer/geometry_widget.h"
#include "cartocrow/renderer/painting_renderer.h"
#include "cartocrow/kinetic_kelp/state_geometry_painting.h"
#include "cartocrow/kinetic_kelp/kinetic_kelp_painting.h"
#include "cartocrow/kinetic_kelp/route_edges.h"

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
        {1, {-75, 2}},
        {2, {-125, 0}},
        {2, {25, 0}},
        {3, {-275, 18}},
        {3, {175, 18}}
    }));

    Settings settings;
    settings.vertexRadius = 10.0;
    settings.edgeWidth = 5.0;

	auto pr = std::make_shared<PaintingRenderer>();
	auto [stateTopology, stateGeometry] = routeEdges(*input, settings, *pr);
	renderer->addPainting(pr, "routeEdges");

//    State state;
//    state.msts.push_back({{0, 1}, {1, 2}});
//    state.msts.push_back({{3, 4}});
//    state.msts.push_back({{5, 6}});
//    state.edgeTopology[{0, 1}] = EdgeTopology(0, 1, {
//        {4, 10.0, 15.0, CGAL::CLOCKWISE},
//        {3, 10.0, 15.0, CGAL::COUNTERCLOCKWISE},
//    });
//    state.edgeTopology[{1, 2}] = EdgeTopology(1, 2, {});
//    state.edgeTopology[{3, 4}] = EdgeTopology(3, 4, {});
//    state.edgeTopology[{5, 6}] = EdgeTopology(5, 6, {
//        {0, 10.0, 15.0, CGAL::CLOCKWISE},
//        {4, 15.0, 20.0, CGAL::CLOCKWISE},
//        {1, 10.0, 15.0, CGAL::CLOCKWISE},
//    });

    auto stateGeometryP = std::make_shared<StateGeometryPainting>(stateGeometry);
    renderer->addPainting(stateGeometryP, "State geometry");

    KineticKelpPainting::DrawSettings ds;
    ds.colors = {CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange};
    ds.markRadius = 1.0;
    ds.strokeWidth = 1.0;
    ds.smoothing = 5.0;

    try {
        auto kelps = std::make_shared<std::vector<Kelp>>();
        stateGeometrytoKelps(*stateGeometry, *input, ds.smoothing, std::back_inserter(*kelps));

        auto kkPainting = std::make_shared<KineticKelpPainting>(kelps, input, ds);
        renderer->addPainting(kkPainting, "KineticKelp");
    } catch (...) {

    }
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    RenderStateDemo demo;
    demo.show();
    app.exec();
}
