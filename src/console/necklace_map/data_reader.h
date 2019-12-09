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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 03-12-2019
*/

#ifndef CONSOLE_NECKLACE_MAP_DATA_READER_H
#define CONSOLE_NECKLACE_MAP_DATA_READER_H

#include <unordered_map>
#include <string>
#include <vector>

#include "console/common/detail/table_reader.h"
#include "geoviz/necklace_map/necklace_element.h"


namespace geoviz
{

class DataReader : public detail::TableReader
{
 public:
  using NecklaceElement = geoviz::necklace_map::NecklaceElement;

 private:
  using LookupTable = std::unordered_map<std::string, size_t>;

 public:
  explicit DataReader(std::vector<NecklaceElement>& elements);

  bool Read(const std::string& filename, const std::string& value_name);

 private:
  std::vector<NecklaceElement>& elements_;

  LookupTable id_to_element_index_;
}; // class DataReader

} // namespace geoviz

#endif //CONSOLE_NECKLACE_MAP_DATA_READER_H
