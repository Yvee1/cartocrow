#include "parse_input.h"
#include "moving_cat_point.h"

#include "../renderer/ipe_reader.h"

#include <ipepath.h>

namespace cartocrow::kinetic_kelp {
std::string getNextLine(std::istream& str) {
	std::string line;
	std::getline(str,line);
	return line;
}

std::vector<std::string> splitIntoTokens(const std::string& line, char delimiter) {
	std::vector<std::string>   result;
	std::stringstream          lineStream(line);
	std::string                cell;

	while(std::getline(lineStream, cell, delimiter))
	{
		result.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty())
	{
		// If there was a trailing comma then add an empty element.
		result.emplace_back("");
	}
	return result;
}

std::vector<CatPoint> parseCatPoints(const std::string& s) {
	std::stringstream ss(s);

	std::vector<CatPoint> result;

	while (ss) {
		auto parts = splitIntoTokens(getNextLine(ss), ' ');
		if (parts.size() <= 1) break;
		if (parts.size() != 3) {
			throw std::runtime_error("Input has incorrect format.");
		}
		result.emplace_back(stoi(parts[0]), Point<Exact>(stod(parts[1]), -stod(parts[2])));
	}

	return result;
}

std::vector<MovingCatPoint> parseIpeAsMovingPoints(const std::filesystem::path& filePath, double secondsBetweenVertices) {
    auto doc = IpeReader::loadIpeFile(filePath);
    auto cascade = doc->cascade();

    if (doc->countPages() == 0) {
        throw std::runtime_error("Cannot read map from an Ipe file with no pages");
    } else if (doc->countPages() > 1) {
        throw std::runtime_error("Cannot read map from an Ipe file with more than one page");
    }

    ipe::Page* page = doc->page(0);

    std::vector<MovingCatPoint> movingCatPoints;

    auto polylineToTrajectory = [secondsBetweenVertices](const Polyline<Inexact>& polyline) {
        Trajectory t;
        for (int i = 0; i < polyline.num_vertices(); ++i) {
            t.m_points.emplace_back(i * secondsBetweenVertices, polyline.vertex(i));
        }
        return t;
    };

    std::map<Color, unsigned int> colorToCat;
    unsigned int numCats = 0;
    for (int i = 0; i < page->count(); ++i) {
        ipe::Object* object = page->object(i);
        ipe::Object::Type type = object->type();
        if (type != ipe::Object::Type::EPath) {
            continue;
        }
        ipe::Path* path = object->asPath();
        ipe::Matrix matrix = path->matrix();
        ipe::Shape ipeShape = path->shape();
        if (ipeShape.countSubPaths() != 1) {
            std::cerr << "Shape consists of multiple subpaths; only parsing the first." << std::endl;
        }
        auto sp = ipeShape.subPath(0);
        if (sp->type() != ipe::SubPath::ECurve) {
            std::cerr << "Subpath is not a curve." << std::endl;
            continue;
        }
        auto c = sp->asCurve();
        auto polyline = IpeReader::convertCurveToPolyline(*c, matrix);

        ipe::Color ipeColor;
        auto fill = path->stroke();
        ipeColor = cascade->find(ipe::Kind::EColor, fill).color();
        Color color = IpeReader::convertIpeColor(ipeColor);

        unsigned int cat;
        if (colorToCat.contains(color)) {
            cat = colorToCat[color];
        } else {
            cat = numCats++;
            colorToCat[color] = cat;
        }

        movingCatPoints.emplace_back(cat, polylineToTrajectory(polyline));
    }

    return movingCatPoints;
}

std::vector<MovingCatPoint> parseCSVAsMovingPoints(const std::filesystem::path& filePath, double seconds) {
    std::stringstream ss;
    std::ifstream file(filePath);
    if (!file.good()) {
        throw std::runtime_error("Failed to read input");
    }
    ss << file.rdbuf();

    std::vector<MovingCatPoint> result;

    while (ss) {
        auto parts = splitIntoTokens(getNextLine(ss), ',');
        if (parts.size() < 2) continue;
        auto cat = stoi(parts[0]);
        assert(parts.size() % 3 == 1);
        auto nSteps = parts.size() / 3;
        Trajectory traj;
        for (int i = 0; i < nSteps; ++i) {
            auto t = stod(parts[3 * i + 1]) * seconds;
            auto x = stod(parts[3 * i + 2]);
            auto y = stod(parts[3 * i + 3]);
            traj.m_points.emplace_back(t, Point<Inexact>(x, y));
        }
        result.emplace_back(cat, std::move(traj));
    }

    return result;
}
}