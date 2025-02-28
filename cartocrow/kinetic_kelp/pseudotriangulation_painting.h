#ifndef CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H
#define CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H

#include "cartocrow/renderer/geometry_painting.h"

#include "pseudotriangulation.h"

namespace cartocrow::kinetic_kelp {
class PseudotriangulationPainting : public renderer::GeometryPainting {
  public:
	PseudotriangulationPainting(std::shared_ptr<PseudotriangulationGeometry> ptg) :
	     m_ptg(std::move(ptg)) {};
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	std::shared_ptr<PseudotriangulationGeometry> m_ptg;
};

class PseudotriangulationCertificatesPainting : public renderer::GeometryPainting {
  public:
	PseudotriangulationCertificatesPainting(std::shared_ptr<Pseudotriangulation> pt, std::shared_ptr<PseudotriangulationGeometry> ptg,
	                            std::shared_ptr<State> state, std::shared_ptr<InputInstance> inputInstance, Settings settings) :
	      m_pt(std::move(pt)), m_ptg(std::move(ptg)), m_state(std::move(state)), m_inputInstance(std::move(inputInstance)), m_settings(std::move(settings)) {};
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	std::shared_ptr<Pseudotriangulation> m_pt;
	std::shared_ptr<PseudotriangulationGeometry> m_ptg;
	std::shared_ptr<State> m_state;
	std::shared_ptr<InputInstance> m_inputInstance;
	Settings m_settings;
};

class CertificateFailurePainting : public renderer::GeometryPainting {
  public:
	CertificateFailurePainting(Pseudotriangulation::TangentEndpointCertificate certificate,
	                           Pseudotriangulation pt, PseudotriangulationGeometry ptg,
	                           State state, std::shared_ptr<StateGeometry> stateGeometry, InputInstance inputInstance, Settings settings) :
	      m_certificate(std::move(certificate)), m_pt(std::move(pt)), m_ptg(std::move(ptg)), m_state(std::move(state)),
	      m_stateGeometry(std::move(stateGeometry)), m_inputInstance(std::move(inputInstance)), m_settings(std::move(settings)) {};
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	Pseudotriangulation::TangentEndpointCertificate m_certificate;
	Pseudotriangulation m_pt;
	PseudotriangulationGeometry m_ptg;
	State m_state;
	std::shared_ptr<StateGeometry> m_stateGeometry;
	InputInstance m_inputInstance;
	Settings m_settings;
};
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H