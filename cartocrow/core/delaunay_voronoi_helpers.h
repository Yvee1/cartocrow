#pragma once

#include "core.h"

namespace cartocrow {
/// Return the sites that define an edge of a (segment) Delaunay graph / triangulation
template <class DG>
std::pair<typename DG::Site_2, typename DG::Site_2> defining_sites(const typename DG::Edge& edge) {
	return {edge.first->vertex(DG::cw(edge.second))->site(),
	        edge.first->vertex(DG::ccw(edge.second))->site()};
}
}