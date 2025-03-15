#ifndef CARTOCROW_POINT_ID_PAINTING_H
#define CARTOCROW_POINT_ID_PAINTING_H

#include "cartocrow/renderer/geometry_painting.h"
#include "input_instance.h"

namespace cartocrow::kinetic_kelp {
class PointIdPainting : public renderer::GeometryPainting {
  public:
	PointIdPainting(std::shared_ptr<InputInstance> input);
	void paint(renderer::GeometryRenderer &renderer) const override;
  private:
	std::shared_ptr<InputInstance> m_input;
};
}

#endif //CARTOCROW_POINT_ID_PAINTING_H
