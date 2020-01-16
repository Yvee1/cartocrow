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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 15-01-2020
*/

#ifndef GEOVIZ_NECKLACE_MAP_NECKLACE_GLYPH_H
#define GEOVIZ_NECKLACE_MAP_NECKLACE_GLYPH_H

#include <memory>

#include "geoviz/common/core_types.h"
#include "geoviz/necklace_map/necklace_interval.h"


namespace geoviz
{
namespace necklace_map
{

struct NecklaceGlyph  // TODO(tvl) rename glyph => bead (including all derived types and methods).
{
  using Ptr = std::shared_ptr<NecklaceGlyph>;

  NecklaceGlyph(const Number& radius_base);

  bool IsValid() const;

  Number radius_base;
  Number angle_rad;

  NecklaceInterval::Ptr interval;  // TODO(tvl) rename feasible.
  Number angle_min_rad;  // TODO(tvl) move out of glyph into glyph-scaler-element...
  Number angle_max_rad;
}; // struct NecklaceGlyph

} // namespace necklace_map
} // namespace geoviz

#endif //GEOVIZ_NECKLACE_MAP_NECKLACE_GLYPH_H
