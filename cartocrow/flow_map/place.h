/*
The Flow Map library implements the algorithmic geo-visualization
method by the same name, developed by Kevin Verbeek, Kevin Buchin,
and Bettina Speckmann at TU Eindhoven
(DOI: 10.1007/s00453-013-9867-z & 10.1109/TVCG.2011.202).
Copyright (C) 2021  Netherlands eScience Center and TU Eindhoven

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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 04-09-2020
*/

#ifndef CARTOCROW_FLOW_MAP_PLACE_H
#define CARTOCROW_FLOW_MAP_PLACE_H

#include <memory>

#include "cartocrow/core/core_types.h"
#include "cartocrow/core/polar_point.h"

namespace cartocrow::flow_map {

/// A place on the flow map.
/**
 * A place has a name, a position, and a numeric value indicating flow that
 * enters this place.
 */
struct Place {
	/// The preferred pointer type for storing or sharing a place.
	using Ptr = std::shared_ptr<Place>;
	/// Construct a new flow map place.
	Place(const std::string& id, const PolarPoint& position);
	/// The ID of this place.
	std::string id;
	/// The position of this place in polar coordinates.
	PolarPoint position;
	/// The amount of flow that enters this place.
	Number flow_in;
};

} // namespace cartocrow::flow_map

#endif //CARTOCROW_FLOW_MAP_PLACE_H