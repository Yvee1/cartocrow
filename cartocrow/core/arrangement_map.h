#pragma once

#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Surface_sweep_2/Arr_default_overlay_traits_base.h>

#include <concepts>
#include <filesystem>

#include "core.h"

namespace cartocrow {

/// An arrangement consisting of polygonal regions.
///
/// This is an \ref Arrangement, or doubly-connected edge list (DCEL)
template <typename TVertexData, typename TEdgeData, typename TFaceData>
using ArrangementMap = CGAL::Arrangement_2<
    CGAL::Arr_segment_traits_2<Exact>,
    CGAL::Arr_extended_dcel<CGAL::Arr_segment_traits_2<Exact>, TVertexData, TEdgeData, TFaceData>>;

template <typename TVertexData, typename TEdgeData, typename TFaceData>
void boundaryMapToArrangementMap(BoundaryMap& map,
                                 ArrangementMap<TVertexData, TEdgeData, TFaceData>& arr);

} // namespace cartocrow

#include "arrangement_map.hpp"