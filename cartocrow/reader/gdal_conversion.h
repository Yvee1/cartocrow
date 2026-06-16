/*
Copyright (C) 2026  TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <ogrsf_frmts.h>
#include "cartocrow/core/core.h"
#include "cartocrow/core/polyline.h"

namespace cartocrow {
PolygonSet<Exact> ogrMultiPolygonToPolygonSet(const OGRMultiPolygon& multiPolygon);
PolygonSet<Exact> ogrPolygonToPolygonSet(const OGRPolygon& ogrPolygon);
Polygon<Exact> ogrLinearRingToPolygon(const OGRLinearRing& ogrLinearRing);
std::vector<Polyline<Exact>> ogrMultiLineStringToMultiPolyline(const OGRMultiLineString& ogrMultiLineString);
Polyline<Exact> ogrLineStringToPolyline(const OGRLineString& ogrLineString);
PolygonWithHoles<Exact> ogrPolygonToPolygonWithHoles(const OGRPolygon& ogrPolygon);
OGRLinearRing polygonToOGRLinearRing(const Polygon<Inexact>& polygon);
OGRPolygon polygonWithHolesToOGRPolygon(const PolygonWithHoles<Inexact>& polygon);
OGRMultiPolygon polygonSetToOGRMultiPolygon(const PolygonSet<Inexact>& polygonSet);
OGRLinearRing polygonToOGRLinearRing(const Polygon<Exact>& polygon);
OGRPolygon polygonWithHolesToOGRPolygon(const PolygonWithHoles<Exact>& polygon);
OGRMultiPolygon polygonSetToOGRMultiPolygon(const PolygonSet<Exact>& polygonSet);
}
