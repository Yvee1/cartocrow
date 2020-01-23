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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 28-11-2019
*/

#ifndef GEOVIZ_NECKLACE_MAP_COMPUTE_SCALE_FACTOR_H
#define GEOVIZ_NECKLACE_MAP_COMPUTE_SCALE_FACTOR_H

#include <unordered_map>
#include <vector>

#include "geoviz/common/core_types.h"
#include "geoviz/necklace_map/necklace.h"


namespace geoviz
{
namespace necklace_map
{

class ComputeScaleFactor
{
 public:
  explicit ComputeScaleFactor(const Number& buffer_rad = 0);

  // Note that elements with value 0 will not be included in the ordering.
  virtual Number operator()(Necklace::Ptr& necklace) = 0;

  Number operator()(std::vector<Necklace::Ptr>& necklaces);

  const Number& max_buffer_rad() const { return max_buffer_rad_; }

 protected:
  Number buffer_rad_;
  Number max_buffer_rad_;
}; // class ComputeScaleFactor


class ComputeScaleFactorFixedOrder : public ComputeScaleFactor
{
 public:
  explicit ComputeScaleFactorFixedOrder(const Number& buffer_rad = 0);

  Number operator()(Necklace::Ptr& necklaces);
}; // class ComputeScaleFactorFixedOrder

} // namespace necklace_map
} // namespace geoviz

#endif //GEOVIZ_NECKLACE_MAP_COMPUTE_SCALE_FACTOR_H