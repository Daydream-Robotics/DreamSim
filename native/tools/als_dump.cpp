// Builds robot-core ALS paths from a JerryIO file and prints the dense samples as CSV.
//
// usage: als_dump <path.jerryio.txt> [sampleSpacing_in]
// output (stdout, field frame, inches/rads): path,s,x,y,heading,curvature,v
//
// Parsing mirrors v5-brain Path::parseFile (cm -> in, duplicate points dropped, one ALS_Path
// per #PATH-POINTS-START block) without the PROS dependencies. v is passed through as-is.
#include "motion/pathing/arclengthSplining.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

static std::vector<std::vector<Waypoint>> parseJerryIO(const std::string& filePath) {
    std::vector<std::vector<Waypoint>> paths;
    std::vector<Waypoint> current;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        return paths;
    }

    bool readingPoints = false;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.find("#PATH-POINTS-START") != std::string::npos) {
            if (!current.empty()) {
                paths.push_back(current);
                current.clear();
            }
            readingPoints = true;
            continue;
        }
        //JSON footer
        if (line.find("#PATH.JERRYIO-DATA") != std::string::npos) {
            break;
        }
        if (!readingPoints || line.empty()) continue;

        double x = 0.0, y = 0.0, v = 0.0, h = 0.0;
        if (std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &x, &y, &v, &h) >= 3) {
            Waypoint wp = {x / 2.54, y / 2.54, v};
            if (!current.empty() && std::hypot(wp.x - current.back().x, wp.y - current.back().y) < 1e-5) {
                continue;
            }
            current.push_back(wp);
        }
    }
    if (!current.empty()) {
        paths.push_back(current);
    }
    return paths;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path.jerryio.txt> [sampleSpacing_in]\n", argv[0]);
        return 2;
    }
    const double spacing = argc > 2 ? std::atof(argv[2]) : 0.25;

    auto rawPaths = parseJerryIO(argv[1]);
    if (rawPaths.empty()) {
        std::fprintf(stderr, "no paths found in %s\n", argv[1]);
        return 1;
    }

    std::printf("path,s,x,y,heading,curvature,v\n");
    for (std::size_t i = 0; i < rawPaths.size(); i++) {
        ALS_Path path;
        if (!path.buildFromPoints(rawPaths[i], spacing)) {
            std::fprintf(stderr, "path %zu failed to build\n", i);
            return 1;
        }
        for (const Sample& s : path.getSamples()) {
            std::printf("%zu,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f\n", i, s.s, s.x, s.y, s.heading, s.curvature, s.v);
        }
    }
    return 0;
}
