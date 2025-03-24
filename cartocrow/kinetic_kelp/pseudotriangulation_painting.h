#ifndef CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H
#define CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H

#include "cartocrow/renderer/geometry_painting.h"
#include <QSpinBox>

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

class PseudotriangulationsPainting : public renderer::GeometryPainting {
public:
    /// kSpinBox is a Qt SpinBox that holds the current category/set that should be painted.
    PseudotriangulationsPainting(const std::shared_ptr<PseudotriangulationGeometries>& ptgs, QSpinBox* kSpinBox) :
        m_kSpinBox(kSpinBox), m_ptgs(ptgs) {};
    void paint(renderer::GeometryRenderer &renderer) const override;

private:
    QSpinBox* m_kSpinBox;
    std::shared_ptr<PseudotriangulationGeometries> m_ptgs;
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

class PseudotriangulationsCertificatesPainting : public renderer::GeometryPainting {
public:
    /// kSpinBox is a Qt SpinBox that holds the current category/set that should be painted.
    PseudotriangulationsCertificatesPainting(std::shared_ptr<Pseudotriangulations> pts, std::shared_ptr<PseudotriangulationGeometries> ptgs,
                                             std::shared_ptr<State> state, std::shared_ptr<InputInstance> inputInstance, Settings settings, QSpinBox* kSpinBox) :
             m_pts(std::move(pts)), m_ptgs(std::move(ptgs)), m_state(std::move(state)), m_inputInstance(std::move(inputInstance)), m_kSpinBox(kSpinBox), m_settings(std::move(settings)) {};
    void paint(renderer::GeometryRenderer &renderer) const override;

private:
    QSpinBox* m_kSpinBox;
    std::shared_ptr<Pseudotriangulations> m_pts;
    std::shared_ptr<PseudotriangulationGeometries> m_ptgs;
    std::shared_ptr<State> m_state;
    std::shared_ptr<InputInstance> m_inputInstance;
    Settings m_settings;
};

class CertificateFailurePainting : public renderer::GeometryPainting {
  public:
	CertificateFailurePainting(std::shared_ptr<Pseudotriangulation::Certificate> certificate, int certificateK,
	                           Pseudotriangulations pts, PseudotriangulationGeometries ptgs,
	                           State state, std::shared_ptr<StateGeometry> stateGeometry, InputInstance inputInstance,
                               Settings settings, QSpinBox* kSpinBox) :
	      m_certificate(std::move(certificate)), m_pts(std::move(pts)), m_ptgs(std::move(ptgs)), m_state(std::move(state)),
	      m_stateGeometry(std::move(stateGeometry)), m_inputInstance(std::move(inputInstance)), m_settings(std::move(settings)),
          m_kSpinBox(kSpinBox), m_certificateK(certificateK) {};
	void paint(renderer::GeometryRenderer &renderer) const override;

  private:
	std::shared_ptr<Pseudotriangulation::Certificate> m_certificate;
    int m_certificateK;
	Pseudotriangulations m_pts;
	PseudotriangulationGeometries m_ptgs;
	State m_state;
	std::shared_ptr<StateGeometry> m_stateGeometry;
	InputInstance m_inputInstance;
	Settings m_settings;
    QSpinBox* m_kSpinBox;
};
}

#endif //CARTOCROW_PSEUDOTRIANGULATION_PAINTING_H