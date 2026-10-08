#ifndef THIN_WALL_IDENTIFIER_HPP
#define THIN_WALL_IDENTIFIER_HPP

#include "GeometryUtils.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Straight_skeleton_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <SFML/Graphics.hpp>
#include <memory>
#include <set>
#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2 = K::Point_2;
using Polygon_2 = CGAL::Polygon_2<K>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<K>;
using Polygon_set_2 = CGAL::Polygon_set_2<K>;
using Ss = CGAL::Straight_skeleton_2<K>;
using SsPtr = std::shared_ptr<Ss>;

struct ThinWallResult {
    Point_2 center;
    double localThickness;
};

class ThinWallIdentifier {
  public:
    double maxThickness = 70.0;
    double pruneFactor = 1.0;
    int samplesPerEdge = 1;
    Polygon_2 polygon;
    std::vector<Polygon_2> holes;
    SsPtr skeleton;
    std::vector<Ss::Halfedge_handle> medialAxis;
    std::vector<ThinWallResult> debugThinWalls;
    void setPolygon(const std::vector<sf::Vector2f> &points);
    void setPolygonWithHoles(
        const std::vector<sf::Vector2f> &outer,
        const std::vector<std::vector<sf::Vector2f>> &holesList);
    bool buildSkeleton();
    std::set<Ss::Halfedge_handle>
    filterSpuriousBranches(double pruneThreshold) const;
    void filterMedialAxis();
    void filterMedialAxisPhase1Only();
    void identifyDebugThinWalls();
    std::vector<Polygon_with_holes_2> getUnifiedThinWalls();
    std::vector<std::vector<Polygon_2>> identifyThinWallChains();
};

#endif
