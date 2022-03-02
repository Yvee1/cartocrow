/*
The Necklace Map library implements the algorithmic
geo-visualization method by the same name, developed by
Bettina Speckmann and Kevin Verbeek at TU Eindhoven
(DOI: 10.1109/TVCG.2010.180 & 10.1142/S021819591550003X).
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
*/

#ifndef CARTOCROW_NECKLACE_MAP_IO_IPE_READER_H
#define CARTOCROW_NECKLACE_MAP_IO_IPE_READER_H

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <ipeattributes.h>
#include <ipebase.h>
#include <ipelib.h>

#include "cartocrow/core/polygon.h"
#include "cartocrow/necklace_map/map_element.h"
#include "cartocrow/necklace_map/necklace.h"

namespace cartocrow::necklace_map {

/// A reader for necklace map input in Ipe format.
/**
 * This reads regions and necklaces from the Ipe file as follows:
 * * stroked (non-filled) paths are interpreted as necklaces;
 * * stroked-and-filled paths are interpreted as regions;
 * * each region is supposed to have one single text label in it, which
 *   indicates the region's name;
 * * to use more than one necklace, put each necklace on a separate layer, along
 *   with the regions that should be mapped to that necklace.
 */
class IpeReader {
  public:
	/// Constructs an Ipe reader.
	IpeReader();

	/// Reads data from the Ipe file with the given filename.
	/**
	 * Stores the data into output parameters \c elements and \c necklaces.
	 */
	bool readFile(const std::filesystem::path& filename,
	              std::vector<necklace_map::MapElement::Ptr>& elements,
	              std::vector<necklace_map::Necklace::Ptr>& necklaces);

  private:
	/// Storage for a label in the input map.
	struct Label {
		/// Position of the label.
		Point position;
		/// The label text.
		std::string text;
		/// Whether we have already matched this label to a region.
		bool matched;
	};
	/// Returns the label from \c labels inside the given region consisting of
	/// \c polygons.
	std::optional<size_t> findLabelInside(std::vector<Polygon_with_holes>& polygons,
	                                      std::vector<Label>& labels);
};

} // namespace cartocrow::necklace_map

#endif //CARTOCROW_NECKLACE_MAP_IO_IPE_READER_H