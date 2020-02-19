/*
The Necklace Map console application implements the algorithmic
geo-visualization method by the same name, developed by
Bettina Speckmann and Kevin Verbeek at TU Eindhoven
(DOI: 10.1109/TVCG.2010.180 & 10.1142/S021819591550003X).
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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 13-02-2020
*/

#ifndef CONSOLE_NECKLACE_MAP_PARSERS_H
#define CONSOLE_NECKLACE_MAP_PARSERS_H

#include <string>

#include <geoviz/necklace_map/parameters.h>


class IntervalTypeParser
{
 public:
  using IntervalType = geoviz::necklace_map::IntervalType;

  static constexpr const char* kCentroid = "centroid";
  static constexpr const char* kWedge = "wedge";

  IntervalTypeParser(IntervalType& type);

  bool operator()(const std::string& str) const;
  std::string Serialize() const;

  IntervalType& type;
}; // class IntervalTypeParser


class OrderTypeParser
{
 public:
  using OrderType = geoviz::necklace_map::OrderType;

  static constexpr const char* kFixed = "fixed";
  static constexpr const char* kAny = "any";
  static constexpr const char* kHeuristic = "heuristic";

  OrderTypeParser(OrderType& type);

  bool operator()(const std::string& str) const;
  std::string Serialize() const;

  OrderType& type;
}; // class OrderTypeParser

#endif //CONSOLE_NECKLACE_MAP_PARSERS_H