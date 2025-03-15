#include "point_id_painting.h"

using namespace cartocrow::renderer;

namespace cartocrow::kinetic_kelp {
PointIdPainting::PointIdPainting(std::shared_ptr<InputInstance> input) :
      m_input(std::move(input)) {}

void PointIdPainting::paint(GeometryRenderer& renderer) const {
	for (int i = 0; i < m_input->catPoints().size(); ++i) {
		const auto& cp = (*m_input)[i];
		renderer.drawText(cp.point, std::to_string(i));
	}
}
}