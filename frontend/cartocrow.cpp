/*
The Necklace Map console application implements the algorithmic
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

Created by tvl (t.vanlankveld@esciencecenter.nl) on 10-09-2019
*/

#include <filesystem>
#include <fstream>

#include <QApplication>

#include <nlohmann/json.hpp>

#include "cartocrow/core/centroid.h"
#include "cartocrow/core/region_map.h"
#include "cartocrow/flow_map/painting.h"
#include "cartocrow/flow_map/spiral_tree.h"
#include "cartocrow/flow_map/spiral_tree_unobstructed_algorithm.h"
#include "cartocrow/isoline_simplification/ipe_isolines.h"
#include "cartocrow/isoline_simplification/isoline_simplifier.h"
#include "cartocrow/isoline_simplification/simple_isoline_painting.h"
#include "cartocrow/simplesets/parse_input.h"
#include "cartocrow/simplesets/settings.h"
#include "cartocrow/simplesets/partition_algorithm.h"
#include "cartocrow/simplesets/drawing_algorithm.h"
#include "cartocrow/necklace_map/circle_necklace.h"
#include "cartocrow/necklace_map/necklace_map.h"
#include "cartocrow/necklace_map/painting.h"
#include "cartocrow/necklace_map/parameters.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include "cartocrow/renderer/svg_renderer.h"

using namespace cartocrow;
using json = nlohmann::json;

int main(int argc, char* argv[]) {
	if (argc != 3 && argc != 4) {
		std::cout << "Usage: cartocrow <project_file> <output_file> [<map_file>]\n";
		std::cout << "where <project_file> is a JSON file describing the map to generate,\n";
		std::cout << "<output_file> is the SVG file to write the output to, and <map_file>\n";
		std::cout << "is an Ipe file containing the underlying map (if necessary for the\n";
		std::cout << "map type generated.\n";
		return 1;
	}

	const std::filesystem::path projectFilename = argv[1];
	const std::filesystem::path outputFilename = argv[2];
	std::string mapFilename = "";
	if (argc == 4) {
		mapFilename = argv[3];
	}
	std::ifstream f(projectFilename);
	json projectData = json::parse(f);

	std::shared_ptr<renderer::GeometryPainting> painting;
	std::shared_ptr<renderer::GeometryPainting> debugPainting;

	if (projectData["type"] == "necklace_map") {
		RegionMap map = ipeToRegionMap(mapFilename);
		auto map_ptr = std::make_shared<RegionMap>(map);

		std::shared_ptr<necklace_map::NecklaceMap> necklaceMap =
		    std::make_shared<necklace_map::NecklaceMap>(map_ptr);
		necklace_map::Parameters& parameters = necklaceMap->parameters();
		parameters.wedge_interval_length_min_rad = 0.1 * M_PI;
		parameters.centroid_interval_length_rad = 0.2 * M_PI;
		parameters.order_type = cartocrow::necklace_map::OrderType::kAny;
		parameters.aversion_ratio = 0.5;

		for (json& n : projectData["necklaces"]) {
			auto necklace = necklaceMap->addNecklace(std::make_unique<necklace_map::CircleNecklace>(
			    Circle<Inexact>(Point<Inexact>(n["shape"]["center"][0], n["shape"]["center"][1]),
			                    std::pow(n["shape"]["radius"].get<double>(), 2))));
			for (std::string b : n["beads"]) {
				necklaceMap->addBead(b, projectData["data"][b], necklace);
			}
		}
		necklaceMap->compute();

		necklace_map::Painting::Options options;
		painting = std::make_shared<necklace_map::Painting>(necklaceMap, options);

	} else if (projectData["type"] == "flow_map") {
		RegionMap map = ipeToRegionMap(projectFilename.parent_path() / projectData["map"]);
		auto map_ptr = std::make_shared<RegionMap>(map);

		// TODO [ws] this is temporary: draw the spiral tree until the flow map
		// is implemented
		Region& root = (*map_ptr)[projectData["root"]];
		std::shared_ptr<flow_map::SpiralTree> tree = std::make_shared<flow_map::SpiralTree>(
		    approximate(centroid(root.shape)), projectData["parameters"]["angle"].get<double>());
		for (auto it = projectData["data"].begin(); it != projectData["data"].end(); ++it) {
			tree->addPlace(it.key(), approximate(centroid((*map_ptr)[it.key()].shape)),
			               it.value().get<double>());
		}
		tree->addShields();

		flow_map::SpiralTreeUnobstructedAlgorithm algorithm(*tree);
		algorithm.run();
		debugPainting = algorithm.debugPainting();

		flow_map::Painting::Options options;
		painting = std::make_shared<flow_map::Painting>(map_ptr, tree, options);

	} else if (projectData["type"] == "isoline_simplification") {
		auto isolines = isoline_simplification::ipeToIsolines(projectFilename.parent_path() / projectData["isolines"]);
		isoline_simplification::IsolineSimplifier simplifier(isolines);
		int target = projectData["target"];
		simplifier.simplify(target);
		painting = std::make_shared<isoline_simplification::SimpleIsolinePainting>(simplifier.m_simplified_isolines);
	} else if (projectData["type"] == "simplesets") {
		// Parse points
		auto filePath = projectFilename.parent_path() / projectData["points"];
		std::ifstream inputStream(filePath, std::ios_base::in);
		if (!inputStream.good()) {
			throw std::runtime_error("Failed to read input");
		}
		std::stringstream buffer;
		buffer << inputStream.rdbuf();
		auto points = simplesets::parseCatPoints(buffer.str());

		// Parse settings
		const auto& pd = projectData;

		simplesets::GeneralSettings gs;
		const auto& pdgs = pd["generalSettings"];
		gs.pointSize = pdgs["pointSize"];
		gs.inflectionLimit = pdgs["inflectionLimit"];
		gs.maxBendAngle = pdgs["maxBendAngle"];
		gs.maxTurnAngle = pdgs["maxTurnAngle"];

		simplesets::DrawSettings ds;
		const auto& pdds = pd["drawSettings"];
		auto pdColors = pdds["colors"];
		std::vector<Color> colors;
		for (const auto& entry : pdColors) {
			std::string hexString = entry.get<std::string>();
			colors.emplace_back(std::strtol(hexString.c_str(), nullptr, 0));
		}
		ds.colors = colors;
		ds.whiten = pdds["whiten"];

		simplesets::PartitionSettings ps;
		const auto& pdps = pd["partitionSettings"];
		ps.banks = pdps["banks"];
		ps.islands = pdps["islands"];
		ps.regularityDelay = pdps["regularityDelay"];
		ps.intersectionDelay = pdps["intersectionDelay"];
		ps.admissibleRadiusFactor = pdps["admissibleRadiusFactor"];

		simplesets::ComputeDrawingSettings cds;
		const auto& pdcds = pd["computeDrawingSettings"];
		cds.smooth = pdcds["smooth"];
		cds.cutoutRadiusFactor = pdcds["cutoutRadiusFactor"];
		cds.smoothingRadiusFactor = pdcds["smoothingRadiusFactor"];

		double cover = pd["cover"];

		// Partition
		auto partitions = simplesets::partition(points, gs, ps, 8 * CGAL::to_double(gs.dilationRadius()));

		// Draw
		simplesets::Partition* thePartition;
		bool found = false;
		for (auto& [time, partition] : partitions) {
			if (time < cover * gs.dilationRadius()) {
				thePartition = &partition;
				found = true;
			}
		}
		simplesets::Partition& partition = found ? (*thePartition) : partitions.front().second;

		bool wellSeparated = true;
		for (const auto& p : points) {
			for (const auto& q : points) {
				if (p.category == q.category) continue;
				if (CGAL::squared_distance(p.point, q.point) < 4 * gs.pointSize * gs.pointSize) {
					wellSeparated = false;
				}
			}
		}
		if (wellSeparated) {
			auto dpd = simplesets::DilatedPatternDrawing(partition, gs, cds);
			auto ssPainting = simplesets::SimpleSetsPainting(dpd, ds);
			auto pr = std::make_shared<renderer::PaintingRenderer>();
			ssPainting.paint(*pr);
			painting = pr;
		} else {
			std::cerr << "Points of different category are too close together; not computing a drawing." << std::endl;
		}
	} else if (projectData["type"] == "chorematic_map") {

	} else {
		std::cerr << "Unknown type \"" << projectData["type"] << "\" specified\n";
	}

	cartocrow::renderer::SvgRenderer renderer(painting);
	renderer.save(outputFilename);
	return 0;
}
