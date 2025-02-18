#ifndef CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H
#define CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H

#include "cartocrow/renderer/geometry_painting.h"

#include "pseudotriangulation.h"

namespace cartocrow::kinetic_kelp {
class PseudotriangulationPainting : public renderer::GeometryPainting {
  public:
	PseudotriangulationPainting(std::shared_ptr<Pseudotriangulation> pt, std::shared_ptr<PseudotriangulationGeometry> ptg) : m_pt(std::move(pt)), m_ptg(std::move(ptg)) {};
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	std::shared_ptr<Pseudotriangulation> m_pt;
	std::shared_ptr<PseudotriangulationGeometry> m_ptg;
};
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H