#pragma once

#include "polyline.h"

// Mirrors CGAL's Polygon_set
namespace cartocrow {
template<class K>
struct PolylineSet {
   std::vector<Polyline<K>> polylines;

   PolylineSet<K> transform(const CGAL::Aff_transformation_2<K>& trans) const {
       PolylineSet<K> transformed;
       for (const auto& pl : polylines) {
           transformed.polylines.push_back(pl.transform(trans));
       }
       return transformed;
   }

   Box bbox() const {
       return CGAL::bbox_2(polylines.begin(), polylines.end());
   }
};

PolylineSet<Inexact> approximate(const PolylineSet<Exact>& pls);
}
