#include "ThinWallIdentifier.hpp"
#include <CGAL/Cartesian_converter.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

static Ss::Halfedge_handle
getDefiningContourEdge(Ss::Halfedge_handle he_skeleton) {
    if (he_skeleton == Ss::Halfedge_handle())
        return Ss::Halfedge_handle();

    auto face = he_skeleton->face();
    if (face == Ss::Face_handle())
        return Ss::Halfedge_handle();

    Ss::Halfedge_handle h = face->halfedge();
    Ss::Halfedge_handle start = h;

    do {
        if (!h->is_bisector())
            return h;
        h = h->next();
    } while (h != start);

    return Ss::Halfedge_handle();
}

void ThinWallIdentifier::setPolygon(const std::vector<sf::Vector2f> &points) {
    polygon.clear();
    holes.clear();
    for (const auto &p : points)
        polygon.push_back(GeoUtils::toCGAL(p));
    if (polygon.size() >= 3) {
        if (polygon.orientation() == CGAL::CLOCKWISE)
            polygon.reverse_orientation();
    }
    skeleton = nullptr;
    medialAxis.clear();
    debugThinWalls.clear();
}

void ThinWallIdentifier::setPolygonWithHoles(
    const std::vector<sf::Vector2f> &outer,
    const std::vector<std::vector<sf::Vector2f>> &holesList) {
    polygon.clear();
    holes.clear();

    for (const auto &p : outer)
        polygon.push_back(GeoUtils::toCGAL(p));
    if (polygon.size() >= 3) {
        if (polygon.orientation() == CGAL::CLOCKWISE)
            polygon.reverse_orientation();
    }

    for (const auto &holePts : holesList) {
        if (holePts.size() < 3)
            continue;
        Polygon_2 hole;
        for (const auto &p : holePts)
            hole.push_back(GeoUtils::toCGAL(p));
        if (hole.orientation() == CGAL::COUNTERCLOCKWISE)
            hole.reverse_orientation();
        holes.push_back(hole);
    }

    skeleton = nullptr;
    medialAxis.clear();
    debugThinWalls.clear();
}

bool ThinWallIdentifier::buildSkeleton() {
    if (polygon.size() < 3)
        return false;
    if (!polygon.is_simple())
        return false;

    std::vector<Polygon_2> validHoles;
    for (size_t i = 0; i < holes.size(); ++i) {
        if (holes[i].size() < 3 || !holes[i].is_simple())
            continue;
        validHoles.push_back(holes[i]);
    }

    try {
        if (validHoles.empty()) {
            skeleton = CGAL::create_interior_straight_skeleton_2(
                polygon.vertices_begin(), polygon.vertices_end());
        } else {
            skeleton = CGAL::create_interior_straight_skeleton_2(
                polygon.vertices_begin(), polygon.vertices_end(),
                validHoles.begin(), validHoles.end());
        }
        return skeleton != nullptr;
    } catch (...) {
        return false;
    }
}

std::set<Ss::Halfedge_handle>
ThinWallIdentifier::filterSpuriousBranches(double pruneThreshold) const {
    std::set<Ss::Halfedge_handle> removedEdges;
    std::set<int> removedIds;

    double squared_threshold = pruneThreshold * pruneThreshold;

    auto markRemoved = [&](Ss::Halfedge_handle h) {
        removedEdges.insert(h);
        removedEdges.insert(h->opposite());
        removedIds.insert(h->id());
        removedIds.insert(h->opposite()->id());
    };

    auto getEffectiveDegree = [&](Ss::Vertex_handle v,
                                  const std::set<int> &blacklist) {
        int deg = 0;
        auto h = v->halfedge();
        auto start = h;
        do {
            if (h->prev()->vertex()->is_skeleton() &&
                h->vertex()->is_skeleton()) {
                if (blacklist.find(h->id()) == blacklist.end()) {
                    deg++;
                }
            }
            h = h->opposite()->prev();
        } while (h != start);
        return deg;
    };

    for (auto vit = skeleton->vertices_begin(); vit != skeleton->vertices_end();
         ++vit) {
        if (!vit->is_contour())
            continue;
        Ss::Halfedge_handle h = vit->halfedge();
        Ss::Halfedge_handle start = h;
        do {
            bool oneIsSkeleton = h->vertex()->is_skeleton() ||
                                 h->opposite()->vertex()->is_skeleton();
            bool bothAreSkeleton = h->vertex()->is_skeleton() &&
                                   h->opposite()->vertex()->is_skeleton();
            if (oneIsSkeleton && !bothAreSkeleton) {
                markRemoved(h);
            }
            h = h->opposite()->prev();
        } while (h != start);
    }

    bool madeCut = true;
    while (madeCut) {
        madeCut = false;

        struct Branch {
            Ss::Vertex_handle junction;
            std::vector<Ss::Halfedge_handle> edges;
            double max_squared_distance;
            bool is_island;
        };
        std::vector<Branch> branches;

        for (auto vit = skeleton->vertices_begin();
             vit != skeleton->vertices_end(); ++vit) {
            if (!vit->is_skeleton())
                continue;

            int deg = getEffectiveDegree(vit, removedIds);

            if (deg >= 3) {
                auto h = vit->halfedge();
                auto start = h;
                do {
                    if (h->prev()->vertex()->is_skeleton() &&
                        h->vertex()->is_skeleton()) {
                        if (removedIds.find(h->id()) == removedIds.end()) {
                            Ss::Halfedge_handle current_edge = h->opposite();
                            Ss::Vertex_handle curr = current_edge->vertex();

                            std::vector<Ss::Halfedge_handle> path_edges;
                            bool reached_tip = false;
                            bool escapes_radius = false;
                            double max_sq = 0.0;

                            while (true) {
                                path_edges.push_back(current_edge);

                                double sq_dist =
                                    CGAL::to_double(CGAL::squared_distance(
                                        vit->point(), curr->point()));
                                if (sq_dist > max_sq)
                                    max_sq = sq_dist;

                                if (sq_dist >= squared_threshold) {
                                    escapes_radius = true;
                                    break;
                                }

                                int curr_deg =
                                    getEffectiveDegree(curr, removedIds);
                                if (curr_deg == 1) {
                                    reached_tip = true;
                                    break;
                                } else if (curr_deg >= 3) {
                                    break;
                                } else {
                                    auto next_h = curr->halfedge();
                                    auto next_start = next_h;
                                    Ss::Halfedge_handle next_edge =
                                        Ss::Halfedge_handle();
                                    do {
                                        if (next_h->prev()
                                                ->vertex()
                                                ->is_skeleton() &&
                                            next_h->vertex()->is_skeleton()) {
                                            if (removedIds.find(next_h->id()) ==
                                                    removedIds.end() &&
                                                next_h->id() !=
                                                    current_edge->id()) {
                                                next_edge = next_h->opposite();
                                                break;
                                            }
                                        }
                                        next_h = next_h->opposite()->prev();
                                    } while (next_h != next_start);

                                    if (next_edge != Ss::Halfedge_handle()) {
                                        current_edge = next_edge;
                                        curr = current_edge->vertex();
                                    } else
                                        break;
                                }
                            }

                            if (reached_tip && !escapes_radius) {
                                Branch b;
                                b.junction = vit;
                                b.edges = path_edges;
                                b.max_squared_distance = max_sq;
                                b.is_island = false;
                                branches.push_back(b);
                            }
                        }
                    }
                    h = h->opposite()->prev();
                } while (h != start);
            }

            else if (deg == 1) {
                Ss::Halfedge_handle current_edge = Ss::Halfedge_handle();
                auto h = vit->halfedge();
                auto start = h;
                do {
                    if (h->prev()->vertex()->is_skeleton() &&
                        h->vertex()->is_skeleton()) {
                        if (removedIds.find(h->id()) == removedIds.end()) {
                            current_edge = h->opposite();
                            break;
                        }
                    }
                    h = h->opposite()->prev();
                } while (h != start);

                if (current_edge != Ss::Halfedge_handle()) {
                    Ss::Vertex_handle curr = current_edge->vertex();
                    std::vector<Ss::Halfedge_handle> path_edges;
                    bool is_island = false;
                    bool escapes_radius = false;
                    double max_sq = 0.0;

                    while (true) {
                        path_edges.push_back(current_edge);

                        double sq_dist = CGAL::to_double(CGAL::squared_distance(
                            vit->point(), curr->point()));
                        if (sq_dist > max_sq)
                            max_sq = sq_dist;

                        if (sq_dist >= squared_threshold) {
                            escapes_radius = true;
                            break;
                        }

                        int curr_deg = getEffectiveDegree(curr, removedIds);
                        if (curr_deg == 1) {
                            is_island = true;
                            break;
                        } else if (curr_deg >= 3) {
                            break;
                        } else {
                            auto next_h = curr->halfedge();
                            auto next_start = next_h;
                            Ss::Halfedge_handle next_edge =
                                Ss::Halfedge_handle();
                            do {
                                if (next_h->prev()->vertex()->is_skeleton() &&
                                    next_h->vertex()->is_skeleton()) {
                                    if (removedIds.find(next_h->id()) ==
                                            removedIds.end() &&
                                        next_h->id() != current_edge->id()) {
                                        next_edge = next_h->opposite();
                                        break;
                                    }
                                }
                                next_h = next_h->opposite()->prev();
                            } while (next_h != next_start);

                            if (next_edge != Ss::Halfedge_handle()) {
                                current_edge = next_edge;
                                curr = current_edge->vertex();
                            } else
                                break;
                        }
                    }

                    if (is_island && !escapes_radius) {
                        Branch b;
                        b.junction = vit;
                        b.edges = path_edges;
                        b.max_squared_distance = max_sq;
                        b.is_island = true;
                        branches.push_back(b);
                    }
                }
            }
        }

        std::sort(branches.begin(), branches.end(),
                  [](const Branch &a, const Branch &b) {
                      return a.max_squared_distance < b.max_squared_distance;
                  });

        for (const auto &b : branches) {
            bool already_removed = false;
            for (auto e : b.edges) {
                if (removedIds.count(e->id())) {
                    already_removed = true;
                    break;
                }
            }
            if (already_removed)
                continue;

            if (b.is_island) {
                for (auto e : b.edges)
                    markRemoved(e);
                madeCut = true;
            } else {
                std::set<int> testIds = removedIds;
                for (auto e : b.edges) {
                    testIds.insert(e->id());
                    testIds.insert(e->opposite()->id());
                }

                int postDeg = getEffectiveDegree(b.junction, testIds);

                if (postDeg >= 1) {
                    for (auto e : b.edges)
                        markRemoved(e);
                    madeCut = true;
                }
            }
        }
    }

    return removedEdges;
}

void ThinWallIdentifier::filterMedialAxis() {
    if (!skeleton)
        return;
    medialAxis.clear();

    double pruneThreshold = pruneFactor * maxThickness;
    std::set<Ss::Halfedge_handle> toRemove =
        filterSpuriousBranches(pruneThreshold);

    for (auto he = skeleton->halfedges_begin(); he != skeleton->halfedges_end();
         ++he) {
        if (!he->vertex()->is_skeleton())
            continue;
        if (!he->opposite()->vertex()->is_skeleton())
            continue;
        if (he->vertex()->id() > he->opposite()->vertex()->id())
            continue;
        if (toRemove.count(he) || toRemove.count(he->opposite()))
            continue;
        medialAxis.push_back(he);
    }
}

void ThinWallIdentifier::filterMedialAxisPhase1Only() {
    if (!skeleton)
        return;
    medialAxis.clear();

    std::set<Ss::Halfedge_handle> edgesToRemove;
    for (auto vit = skeleton->vertices_begin(); vit != skeleton->vertices_end();
         ++vit) {
        if (!vit->is_contour())
            continue;
        Ss::Halfedge_handle h = vit->halfedge();
        Ss::Halfedge_handle start = h;
        do {
            bool oneIsSkeleton = h->vertex()->is_skeleton() ||
                                 h->opposite()->vertex()->is_skeleton();
            bool bothAreSkeleton = h->vertex()->is_skeleton() &&
                                   h->opposite()->vertex()->is_skeleton();
            if (oneIsSkeleton && !bothAreSkeleton) {
                edgesToRemove.insert(h);
                edgesToRemove.insert(h->opposite());
            }
            h = h->opposite()->prev();
        } while (h != start);
    }

    for (auto he = skeleton->halfedges_begin(); he != skeleton->halfedges_end();
         ++he) {
        if (!he->vertex()->is_skeleton())
            continue;
        if (!he->opposite()->vertex()->is_skeleton())
            continue;
        if (he->vertex()->id() > he->opposite()->vertex()->id())
            continue;
        if (edgesToRemove.count(he) || edgesToRemove.count(he->opposite()))
            continue;
        medialAxis.push_back(he);
    }
}

void ThinWallIdentifier::identifyDebugThinWalls() {
    debugThinWalls.clear();
    if (medialAxis.empty())
        return;

    double halfThreshold = maxThickness / 2.0;

    for (const auto &he : medialAxis) {
        auto v_source = he->opposite()->vertex();
        auto v_target = he->vertex();

        Point_2 p1 = v_source->point();
        Point_2 p2 = v_target->point();

        double t_source = v_source->time();
        double t_target = v_target->time();

        for (int i = 0; i <= samplesPerEdge; ++i) {
            double t = (double)i / samplesPerEdge;

            Point_2 center(p1.x() + (p2.x() - p1.x()) * t,
                           p1.y() + (p2.y() - p1.y()) * t);

            double r = t_source + t * (t_target - t_source);

            if (r < halfThreshold) {
                ThinWallResult res;
                res.center = center;
                res.localThickness = r * 2.0;
                debugThinWalls.push_back(res);
            }
        }
    }
}

static bool projectOnDefiningContour(const Point_2 &C, Ss::Halfedge_handle he,
                                     Point_2 &out) {
    Ss::Halfedge_handle contour = getDefiningContourEdge(he);
    if (contour == Ss::Halfedge_handle())
        return false;

    out = GeoUtils::projectPointOnSegment(
        C, contour->opposite()->vertex()->point(), contour->vertex()->point());
    return true;
}

static double thresholdParam(double t_src, double t_tgt, double halfThreshold) {
    return (halfThreshold - t_src) / (t_tgt - t_src);
}

std::vector<std::vector<Polygon_2>>
ThinWallIdentifier::identifyThinWallChains() {
    std::vector<std::vector<Polygon_2>> resultChains;
    if (!skeleton || medialAxis.empty())
        return resultChains;

    const double halfThreshold = maxThickness / 2.0;

    auto getDegree = [&](Ss::Vertex_handle v) {
        int deg = 0;
        auto h = v->halfedge();
        auto start = h;
        do {
            if (h->prev()->vertex()->is_skeleton() &&
                h->vertex()->is_skeleton())
                deg++;
            h = h->opposite()->prev();
        } while (h != start);
        return deg;
    };

    struct ThinPart {
        Ss::Halfedge_handle he;
        double t0, t1;
    };
    std::map<int, ThinPart> thin_parts;

    for (const auto &he : medialAxis) {
        auto v_src = he->opposite()->vertex();
        auto v_tgt = he->vertex();
        double t_src = v_src->time(), t_tgt = v_tgt->time();
        double t0 = 0.0, t1 = 1.0;
        bool hasThinPart = false;

        if (t_src < halfThreshold && t_tgt < halfThreshold) {
            hasThinPart = true;
            t0 = 0.0;
            t1 = 1.0;
        } else if (t_src < halfThreshold && t_tgt >= halfThreshold) {
            hasThinPart = true;
            t0 = 0.0;
            t1 = thresholdParam(t_src, t_tgt, halfThreshold);
        } else if (t_src >= halfThreshold && t_tgt < halfThreshold) {
            hasThinPart = true;
            t0 = thresholdParam(t_src, t_tgt, halfThreshold);
            t1 = 1.0;
        }

        if (hasThinPart && std::abs(t1 - t0) > 1e-9) {
            thin_parts[he->id()] = {he, t0, t1};
            thin_parts[he->opposite()->id()] = {he->opposite(), 1.0 - t1,
                                                1.0 - t0};
        }
    }

    std::set<int> visited;

    for (const auto &base_he : medialAxis) {
        if (visited.count(base_he->id()) ||
            visited.count(base_he->opposite()->id()))
            continue;
        if (thin_parts.find(base_he->id()) == thin_parts.end())
            continue;

        std::vector<ThinPart> chain;
        chain.push_back(thin_parts[base_he->id()]);
        visited.insert(base_he->id());
        visited.insert(base_he->opposite()->id());

        while (true) {
            auto curr = chain.back();
            if (curr.t1 < 1.0 - 1e-9)
                break;
            auto v = curr.he->vertex();
            if (getDegree(v) != 2)
                break;

            Ss::Halfedge_handle next_he;
            auto h = v->halfedge();
            auto start = h;
            do {
                if (h->prev()->vertex()->is_skeleton() &&
                    h->vertex()->is_skeleton()) {
                    if (h->id() != curr.he->id())
                        next_he = h->opposite();
                }
                h = h->opposite()->prev();
            } while (h != start);

            if (next_he != Ss::Halfedge_handle() &&
                thin_parts.count(next_he->id()) &&
                !visited.count(next_he->id())) {
                chain.push_back(thin_parts[next_he->id()]);
                visited.insert(next_he->id());
                visited.insert(next_he->opposite()->id());
            } else
                break;
        }

        while (true) {
            auto curr = chain.front();
            if (curr.t0 > 1e-9)
                break;
            auto v = curr.he->opposite()->vertex();
            if (getDegree(v) != 2)
                break;

            Ss::Halfedge_handle prev_he;
            auto h = v->halfedge();
            auto start = h;
            do {
                if (h->prev()->vertex()->is_skeleton() &&
                    h->vertex()->is_skeleton()) {
                    if (h->opposite()->id() != curr.he->id())
                        prev_he = h;
                }
                h = h->opposite()->prev();
            } while (h != start);

            if (prev_he != Ss::Halfedge_handle() &&
                thin_parts.count(prev_he->id()) &&
                !visited.count(prev_he->id())) {
                chain.insert(chain.begin(), thin_parts[prev_he->id()]);
                visited.insert(prev_he->id());
                visited.insert(prev_he->opposite()->id());
            } else
                break;
        }

        auto is_same = [](const Point_2 &a, const Point_2 &b) {
            return CGAL::squared_distance(a, b) < 1e-7;
        };

        std::vector<Point_2> pts;
        auto add_pt = [&](const Point_2 &p) {
            if (pts.empty() || !is_same(pts.back(), p))
                pts.push_back(p);
        };

        auto first = chain.front();
        auto last = chain.back();

        Point_2 ps_start = first.he->opposite()->vertex()->point();
        Point_2 pt_start = first.he->vertex()->point();
        Point_2 P =
            (first.t0 <= 0.0)
                ? ps_start
                : Point_2(
                      ps_start.x() + (pt_start.x() - ps_start.x()) * first.t0,
                      ps_start.y() + (pt_start.y() - ps_start.y()) * first.t0);

        Point_2 ps_end = last.he->opposite()->vertex()->point();
        Point_2 pt_end = last.he->vertex()->point();
        Point_2 P_prime =
            (last.t1 >= 1.0)
                ? pt_end
                : Point_2(ps_end.x() + (pt_end.x() - ps_end.x()) * last.t1,
                          ps_end.y() + (pt_end.y() - ps_end.y()) * last.t1);

        Point_2 Pe, Pd, Pe_prime, Pd_prime;
        if (!projectOnDefiningContour(P, first.he, Pe))
            continue;
        if (!projectOnDefiningContour(P, first.he->opposite(), Pd))
            continue;
        if (!projectOnDefiningContour(P_prime, last.he, Pe_prime))
            continue;
        if (!projectOnDefiningContour(P_prime, last.he->opposite(), Pd_prime))
            continue;

        add_pt(P);
        add_pt(Pe);

        for (size_t i = 0; i < chain.size() - 1; ++i) {
            auto c1 = getDefiningContourEdge(chain[i].he);
            auto c2 = getDefiningContourEdge(chain[i + 1].he);

            if (c1 != c2 && c1 != Ss::Halfedge_handle() &&
                c2 != Ss::Halfedge_handle()) {
                Point_2 L_p1 = c1->vertex()->point();
                Point_2 L_p2 = c1->opposite()->vertex()->point();
                Point_2 R_p1 = c2->vertex()->point();
                Point_2 R_p2 = c2->opposite()->vertex()->point();

                Point_2 V_orig;
                bool found = false;
                if (is_same(L_p1, R_p1) || is_same(L_p1, R_p2)) {
                    V_orig = L_p1;
                    found = true;
                } else if (is_same(L_p2, R_p1) || is_same(L_p2, R_p2)) {
                    V_orig = L_p2;
                    found = true;
                }

                if (found)
                    add_pt(V_orig);
            }
        }

        add_pt(Pe_prime);
        add_pt(P_prime);
        add_pt(Pd_prime);

        for (int i = (int)chain.size() - 1; i > 0; --i) {
            auto c1 = getDefiningContourEdge(chain[i].he->opposite());
            auto c2 = getDefiningContourEdge(chain[i - 1].he->opposite());

            if (c1 != c2 && c1 != Ss::Halfedge_handle() &&
                c2 != Ss::Halfedge_handle()) {
                Point_2 L_p1 = c1->vertex()->point();
                Point_2 L_p2 = c1->opposite()->vertex()->point();
                Point_2 R_p1 = c2->vertex()->point();
                Point_2 R_p2 = c2->opposite()->vertex()->point();

                Point_2 V_orig;
                bool found = false;
                if (is_same(L_p1, R_p1) || is_same(L_p1, R_p2)) {
                    V_orig = L_p1;
                    found = true;
                } else if (is_same(L_p2, R_p1) || is_same(L_p2, R_p2)) {
                    V_orig = L_p2;
                    found = true;
                }

                if (found)
                    add_pt(V_orig);
            }
        }

        add_pt(Pd);

        if (pts.size() > 1 && is_same(pts.front(), pts.back()))
            pts.pop_back();

        Polygon_2 poly;
        for (const auto &p : pts)
            poly.push_back(p);

        if (poly.size() >= 3 && poly.is_simple()) {
            if (poly.orientation() == CGAL::CLOCKWISE)
                poly.reverse_orientation();
            resultChains.push_back({poly});
        }
    }

    return resultChains;
}

std::vector<Polygon_with_holes_2> ThinWallIdentifier::getUnifiedThinWalls() {
    std::vector<Polygon_with_holes_2> result;

    auto chains = identifyThinWallChains();
    if (chains.empty())
        return result;

    using EK = CGAL::Exact_predicates_exact_constructions_kernel;
    using EPolygon_2 = CGAL::Polygon_2<EK>;
    using EPwh_2 = CGAL::Polygon_with_holes_2<EK>;
    using EPolygonSet_2 = CGAL::Polygon_set_2<EK>;

    CGAL::Cartesian_converter<K, EK> to_exact;
    EPolygonSet_2 exact_pset;

    for (const auto &chain_group : chains) {
        for (const auto &poly : chain_group) {
            if (!poly.is_simple())
                continue;

            Polygon_2 temp_poly = poly;
            if (temp_poly.orientation() == CGAL::CLOCKWISE)
                temp_poly.reverse_orientation();

            EPolygon_2 epoly;
            for (auto vit = temp_poly.vertices_begin();
                 vit != temp_poly.vertices_end(); ++vit)
                epoly.push_back(to_exact(*vit));

            if (!epoly.is_simple())
                continue;
            if (epoly.orientation() == CGAL::CLOCKWISE)
                epoly.reverse_orientation();

            try {
                exact_pset.join(epoly);
            } catch (const std::exception &) {
            }
        }
    }

    std::list<EPwh_2> exact_unified;
    exact_pset.polygons_with_holes(std::back_inserter(exact_unified));

    auto toInexactPt = [](const EK::Point_2 &p) -> Point_2 {
        return Point_2(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    };

    auto convertPoly = [&](const EPolygon_2 &ep) -> Polygon_2 {
        Polygon_2 p;
        for (auto vit = ep.vertices_begin(); vit != ep.vertices_end(); ++vit)
            p.push_back(toInexactPt(*vit));
        return p;
    };

    for (const auto &epwh : exact_unified) {
        Polygon_2 outer = convertPoly(epwh.outer_boundary());
        Polygon_with_holes_2 pwh(outer);
        for (auto hit = epwh.holes_begin(); hit != epwh.holes_end(); ++hit)
            pwh.add_hole(convertPoly(*hit));
        result.push_back(std::move(pwh));
    }

    return result;
}
