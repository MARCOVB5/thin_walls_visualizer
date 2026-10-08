#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2 = K::Point_2;
using Segment_2 = K::Segment_2;

namespace GeoUtils {
inline sf::Vector2f toSFML(const Point_2 &p) {
    return sf::Vector2f(static_cast<float>(p.x()), static_cast<float>(p.y()));
}

inline Point_2 toCGAL(const sf::Vector2f &v) { return Point_2(v.x, v.y); }

inline Point_2 projectPointOnSegment(const Point_2 &p, const Point_2 &a,
                                     const Point_2 &b) {
    double abx = b.x() - a.x();
    double aby = b.y() - a.y();
    double apx = p.x() - a.x();
    double apy = p.y() - a.y();

    double lenSq = abx * abx + aby * aby;
    if (lenSq == 0)
        return a;

    double t = (apx * abx + apy * aby) / lenSq;
    t = std::max(0.0, std::min(1.0, t));

    return Point_2(a.x() + t * abx, a.y() + t * aby);
}

inline bool loadPolygonFromFile(const std::string &filepath,
                                std::vector<sf::Vector2f> &outer,
                                std::vector<std::vector<sf::Vector2f>> &holes) {
    std::ifstream file(filepath);
    if (!file.is_open())
        return false;

    std::vector<sf::Vector2f> newOuter;
    std::vector<std::vector<sf::Vector2f>> newHoles;
    std::vector<sf::Vector2f> *target = nullptr;

    const char *ws = " \t\r\n";
    std::string line;
    while (std::getline(file, line)) {
        const auto first = line.find_first_not_of(ws);
        if (first == std::string::npos)
            continue;
        line = line.substr(first, line.find_last_not_of(ws) - first + 1);

        if (line[0] == '#')
            continue;
        if (line == "OUTER") {
            newOuter.clear();
            target = &newOuter;
            continue;
        }
        if (line == "HOLE") {
            newHoles.emplace_back();
            target = &newHoles.back();
            continue;
        }
        if (!target)
            continue;

        std::istringstream ss(line);
        float x, y;
        char comma;
        if ((ss >> x >> comma >> y) && comma == ',')
            target->push_back({x, y});
    }

    if (newOuter.size() < 3)
        return false;

    newHoles.erase(std::remove_if(newHoles.begin(), newHoles.end(),
                                  [](const std::vector<sf::Vector2f> &h) {
                                      return h.size() < 3;
                                  }),
                   newHoles.end());

    outer = std::move(newOuter);
    holes = std::move(newHoles);
    return true;
}

} 

#endif
