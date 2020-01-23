/*
The Necklace Map library implements the algorithmic geo-visualization
method by the same name, developed by Bettina Speckmann and Kevin Verbeek
at TU Eindhoven (DOI: 10.1109/TVCG.2010.180 & 10.1142/S021819591550003X).
Copyright (C) 2019  Netherlands eScience Center and TU Eindhoven

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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 23-01-2020
*/

#ifndef GEOVIZ_NECKLACE_MAP_DETAIL_CYCLE_NODE_H
#define GEOVIZ_NECKLACE_MAP_DETAIL_CYCLE_NODE_H

#include <memory>

#include "geoviz/common/core_types.h"
#include "geoviz/necklace_map/bead.h"


namespace geoviz
{
namespace necklace_map
{
namespace detail
{

// A node to cycle through the beads.
// As opposed to beads, these nodes may have a feasible interval completely outside [0, 2pi).
// This means that they can be used to cycle through the nodes multiple times in order.
struct BeadCycleNode
{
  using Ptr = std::shared_ptr<BeadCycleNode>;

  BeadCycleNode(const Bead::Ptr& bead);

  Bead::Ptr bead;

  // Note that unlike the bead's feasible interval, these can be larger than 2*PI.
  Number interval_cw_rad;
  Number interval_ccw_rad;
}; // struct BeadCycleNode

} // namespace detail
} // namespace necklace_map
} // namespace geoviz

#endif //GEOVIZ_NECKLACE_MAP_DETAIL_CYCLE_NODE_H