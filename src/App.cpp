#include "App.hpp"
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/partition_2.h>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <list>
#include <numeric>
#include <string>
#ifndef ASSETS_DIR
#define ASSETS_DIR "assets"
#endif

App::App(const std::string &polyName)
    : window(sf::VideoMode({1024, 768}), "thin walls"), outerClosed(false),
      drawingHole(false), unifyPolygons(false), drawingMode(DrawingMode::OUTER),
      currentView(ViewMode::ANALYSIS), hasFont(false), draggingSlider(false),
      draggingPruneSlider(false) {
    window.setFramerateLimit(60);
    btnReset.setSize({100, 40});
    btnReset.setPosition({10, 710});
    btnReset.setFillColor(sf::Color(70, 70, 70));
    btnReset.setOutlineColor(sf::Color::White);
    btnReset.setOutlineThickness(1);

    btnAddHole.setSize({110, 40});
    btnAddHole.setPosition({120, 710});
    btnAddHole.setFillColor(sf::Color(70, 70, 70));
    btnAddHole.setOutlineColor(sf::Color::Yellow);
    btnAddHole.setOutlineThickness(1);

    btnToggleView.setSize({140, 40});
    btnToggleView.setPosition({1024 - 300, 710});
    btnToggleView.setFillColor(sf::Color(70, 70, 70));
    btnToggleView.setOutlineColor(sf::Color::Cyan);
    btnToggleView.setOutlineThickness(1);

    btnToggleArea.setSize({140, 40});
    btnToggleArea.setPosition({1024 - 150, 710});
    btnToggleArea.setFillColor(sf::Color(70, 70, 70));
    btnToggleArea.setOutlineColor(sf::Color(255, 50, 50));
    btnToggleArea.setOutlineThickness(1);

    sliderBg.setSize({200, 5});
    sliderBg.setPosition({362, 730});
    sliderBg.setFillColor(sf::Color(100, 100, 100));

    sliderHandle.setSize({10, 20});
    sliderHandle.setOrigin({5, 10});
    sliderHandle.setFillColor(sf::Color::Cyan);

    identifier.maxThickness = 20.0;
    updateSliderPos();

    sliderPruneBg.setSize({160, 5});
    sliderPruneBg.setPosition({840, 20});
    sliderPruneBg.setFillColor(sf::Color(100, 100, 100));

    sliderPruneHandle.setSize({10, 20});
    sliderPruneHandle.setOrigin({5, 10});
    sliderPruneHandle.setFillColor(sf::Color(255, 160, 0));

    identifier.pruneFactor = 1.0;
    updatePruneSliderPos();
    const std::filesystem::path fontPath =
        std::filesystem::path(ASSETS_DIR) / "fonts" / "DejaVuSans.ttf";

    hasFont = font.openFromFile(fontPath);

    if (!polyName.empty()) {
        loadedFileName = polyName;
        loadFromFile(polyName);
    }
}

void App::updateSliderPos() {
    float ratio = (static_cast<float>(identifier.maxThickness) - 5.0f) / 95.0f;
    sliderHandle.setPosition({362.0f + ratio * 200.0f, 732.5f});
}

void App::updatePruneSliderPos() {
    float ratio = static_cast<float>(identifier.pruneFactor) / 10.0f;
    sliderPruneHandle.setPosition({840.0f + ratio * 160.0f, 22.5f});
}

void App::updatePruneFromSlider(float x) {
    float localX = std::max(0.0f, std::min(x - 840.0f, 160.0f));
    float ratio = localX / 160.0f;
    identifier.pruneFactor = static_cast<double>(ratio * 10.0f);
    sliderPruneHandle.setPosition({840.0f + localX, 22.5f});

    if (outerClosed && identifier.skeleton) {
        identifier.filterMedialAxis();
        identifier.identifyDebugThinWalls();
    }
}

void App::updateThicknessFromSlider(float x) {
    float localX = std::max(0.0f, std::min(x - 362.0f, 200.0f));
    float ratio = localX / 200.0f;
    identifier.maxThickness = 5.0 + ratio * 95.0;
    sliderHandle.setPosition({362.0f + localX, 732.5f});

    if (outerClosed && identifier.skeleton) {
        identifier.filterMedialAxis();
        identifier.identifyDebugThinWalls();
    }
}

void App::rebuildAndAnalyze() {
    identifier.setPolygonWithHoles(outerPoints, completedHoles);
    if (identifier.buildSkeleton()) {
        identifier.filterMedialAxis();
        identifier.identifyDebugThinWalls();
    }
}

void App::run() {
    while (window.isOpen()) {
        handleEvents();
        render();
    }
}

void App::handleEvents() {
    while (const auto event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>())
            window.close();

        if (const auto *mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
            sf::Vector2f mPos(static_cast<float>(mouse->position.x),
                              static_cast<float>(mouse->position.y));

            if (mouse->button == sf::Mouse::Button::Left) {

                sf::FloatRect handleRect = sliderHandle.getGlobalBounds();
                handleRect.position.x -= 5;
                handleRect.size.x += 10;
                if (handleRect.contains(mPos)) {
                    draggingSlider = true;
                    continue;
                }

                sf::FloatRect pruneRect = sliderPruneHandle.getGlobalBounds();
                pruneRect.position.x -= 5;
                pruneRect.size.x += 10;
                if (pruneRect.contains(mPos)) {
                    draggingPruneSlider = true;
                    continue;
                }

                if (btnReset.getGlobalBounds().contains(mPos)) {
                    resetDrawing();
                    continue;
                }

                if (btnAddHole.getGlobalBounds().contains(mPos)) {
                    if (outerClosed) {
                        currentHolePoints.clear();
                        drawingHole = true;
                        drawingMode = DrawingMode::HOLE;
                    }
                    continue;
                }

                if (btnToggleView.getGlobalBounds().contains(mPos)) {
                    if (outerClosed)
                        currentView = (currentView == ViewMode::SKELETON)
                                          ? ViewMode::ANALYSIS
                                          : ViewMode::SKELETON;
                    continue;
                }

                if (btnToggleArea.getGlobalBounds().contains(mPos)) {
                    if (outerClosed)
                        currentView = (currentView == ViewMode::AREA)
                                          ? ViewMode::ANALYSIS
                                          : ViewMode::AREA;
                    continue;
                }

                if (drawingMode == DrawingMode::HOLE && drawingHole) {
                    if (currentHolePoints.size() > 2) {
                        float d = std::pow(mPos.x - currentHolePoints[0].x, 2) +
                                  std::pow(mPos.y - currentHolePoints[0].y, 2);
                        if (d < 225.0f) {
                            completedHoles.push_back(currentHolePoints);
                            currentHolePoints.clear();
                            drawingHole = false;
                            drawingMode = DrawingMode::OUTER;
                            rebuildAndAnalyze();
                            continue;
                        }
                    }
                    currentHolePoints.push_back(mPos);

                } else if (!outerClosed) {
                    if (outerPoints.size() > 2) {
                        float d = std::pow(mPos.x - outerPoints[0].x, 2) +
                                  std::pow(mPos.y - outerPoints[0].y, 2);
                        if (d < 225.0f) {
                            outerClosed = true;
                            rebuildAndAnalyze();
                            continue;
                        }
                    }
                    outerPoints.push_back(mPos);
                }
            }
        }

        if (event->getIf<sf::Event::MouseButtonReleased>()) {
            draggingSlider = false;
            draggingPruneSlider = false;
        }

        if (const auto *move = event->getIf<sf::Event::MouseMoved>()) {
            if (draggingSlider)
                updateThicknessFromSlider(static_cast<float>(move->position.x));
            if (draggingPruneSlider)
                updatePruneFromSlider(static_cast<float>(move->position.x));
        }

        if (const auto *key = event->getIf<sf::Event::KeyPressed>()) {
            if (key->code == sf::Keyboard::Key::R)
                resetDrawing();
            if (key->code == sf::Keyboard::Key::H && outerClosed) {
                currentHolePoints.clear();
                drawingHole = true;
                drawingMode = DrawingMode::HOLE;
            }
            if (key->code == sf::Keyboard::Key::Escape && drawingHole) {
                currentHolePoints.clear();
                drawingHole = false;
                drawingMode = DrawingMode::OUTER;
            }

            if (key->code == sf::Keyboard::Key::U) {
                unifyPolygons = !unifyPolygons;
            }

            if (key->code == sf::Keyboard::Key::Equal && outerClosed &&
                identifier.skeleton) {
                identifier.filterMedialAxisPhase1Only();
                identifier.identifyDebugThinWalls();
            }

            if (key->code == sf::Keyboard::Key::L && !loadedFileName.empty())
                loadFromFile(loadedFileName);
        }
    }
}

void App::drawPolygonOutline(const std::vector<sf::Vector2f> &pts, bool closed,
                             sf::Color color, bool dimmed) {
    if (pts.empty())
        return;
    sf::Color c =
        dimmed ? sf::Color(color.r / 3, color.g / 3, color.b / 3, 180) : color;

    std::vector<sf::Vertex> lines;
    for (size_t i = 0; i < pts.size(); ++i) {
        lines.push_back(sf::Vertex{pts[i], c});
        if (i + 1 < pts.size())
            lines.push_back(sf::Vertex{pts[i + 1], c});
        else if (closed)
            lines.push_back(sf::Vertex{pts[0], c});
    }
    window.draw(lines.data(), lines.size(), sf::PrimitiveType::Lines);
    for (const auto &p : pts) {
        sf::CircleShape pt(3.f);
        pt.setOrigin({3.f, 3.f});
        pt.setPosition(p);
        pt.setFillColor(c);
        window.draw(pt);
    }
}

void App::render() {
    window.clear(sf::Color(30, 30, 35));

    sf::RectangleShape line;
    line.setFillColor(sf::Color(50, 50, 60));
    for (int y = 0; y < 768; y += 40) {
        line.setSize({1024.f, 1.f});
        line.setPosition({0.f, (float)y});
        window.draw(line);
    }
    for (int x = 0; x < 1024; x += 40) {
        line.setSize({1.f, 768.f});
        line.setPosition({(float)x, 0.f});
        window.draw(line);
    }

    bool dimContour = (currentView != ViewMode::ANALYSIS);

    if (!outerPoints.empty()) {
        drawPolygonOutline(outerPoints, outerClosed, sf::Color::White,
                           dimContour);
        if (!outerClosed && drawingMode == DrawingMode::OUTER) {
            sf::Vector2i m = sf::Mouse::getPosition(window);
            std::vector<sf::Vertex> guide = {
                sf::Vertex{outerPoints.back(), sf::Color::Yellow},
                sf::Vertex{sf::Vector2f((float)m.x, (float)m.y),
                           sf::Color::Yellow}};
            window.draw(guide.data(), 2, sf::PrimitiveType::Lines);
        }
    }

    for (const auto &hole : completedHoles)
        drawPolygonOutline(hole, true, sf::Color::Yellow, dimContour);

    if (drawingHole && !currentHolePoints.empty()) {
        drawPolygonOutline(currentHolePoints, false, sf::Color(255, 220, 50));
        sf::Vector2i m = sf::Mouse::getPosition(window);
        std::vector<sf::Vertex> guide = {
            sf::Vertex{currentHolePoints.back(), sf::Color(255, 220, 50)},
            sf::Vertex{sf::Vector2f((float)m.x, (float)m.y),
                       sf::Color(255, 220, 50)}};
        window.draw(guide.data(), 2, sf::PrimitiveType::Lines);
        if (currentHolePoints.size() > 2) {
            sf::CircleShape ci(10.f);
            ci.setOrigin({10.f, 10.f});
            ci.setPosition(currentHolePoints[0]);
            ci.setFillColor(sf::Color(255, 220, 50, 80));
            ci.setOutlineColor(sf::Color(255, 220, 50));
            ci.setOutlineThickness(1.5f);
            window.draw(ci);
        }
    }

    if (currentView == ViewMode::SKELETON) {
        if (outerClosed && !identifier.medialAxis.empty()) {
            std::vector<sf::Vertex> skLines;
            for (const auto &seg : identifier.medialAxis) {
                auto pSrc = seg->opposite()->vertex()->point();
                auto pTgt = seg->vertex()->point();

                skLines.push_back(
                    sf::Vertex{GeoUtils::toSFML(pSrc), sf::Color::Cyan});
                skLines.push_back(
                    sf::Vertex{GeoUtils::toSFML(pTgt), sf::Color::Cyan});
            }
            window.draw(skLines.data(), skLines.size(),
                        sf::PrimitiveType::Lines);
        }
    }

    else if (currentView == ViewMode::AREA) {
        if (outerClosed && identifier.skeleton) {

            // auto chains = identifier.identifyThinWallChains();
            std::vector<sf::Vertex> allTriangles;
            std::vector<sf::CircleShape> allVertices;

            std::vector<sf::Color> palette = {
                sf::Color(220, 50, 50, 160),   // red
                sf::Color(50, 220, 50, 160),   // green
                sf::Color(50, 100, 255, 160),  // blue
                sf::Color(220, 220, 50, 160),  // yellow
                sf::Color(220, 50, 220, 160),  // magenta
                sf::Color(50, 220, 220, 160),  // cyan
                sf::Color(255, 140, 0, 160),   // dark orange
                sf::Color(148, 0, 211, 160),   // violet
                sf::Color(0, 150, 136, 160),   // teal
                sf::Color(255, 105, 180, 160), // pink
                sf::Color(154, 205, 50, 160),  // yelloish green
                sf::Color(160, 82, 45, 160)    // brown
            };

            auto triangulateAndAppend = [&](Polygon_2 poly,
                                            sf::Color fillColor) {
                if (poly.size() < 3)
                    return;
                if (!poly.is_simple())
                    return;
                if (poly.orientation() == CGAL::CLOCKWISE)
                    poly.reverse_orientation();

                using PartitionPoly = CGAL::Polygon_2<K, std::list<Point_2>>;
                std::list<PartitionPoly> partition_polys;

                try {
                    CGAL::approx_convex_partition_2(
                        poly.vertices_begin(), poly.vertices_end(),
                        std::back_inserter(partition_polys));

                    for (const auto &convex_poly : partition_polys) {
                        if (convex_poly.size() < 3)
                            continue;

                        auto vit = convex_poly.vertices_begin();
                        sf::Vector2f v0 = GeoUtils::toSFML(*vit);
                        ++vit;
                        sf::Vector2f v_prev = GeoUtils::toSFML(*vit);
                        ++vit;

                        for (; vit != convex_poly.vertices_end(); ++vit) {
                            sf::Vector2f v_curr = GeoUtils::toSFML(*vit);
                            allTriangles.push_back(sf::Vertex{v0, fillColor});
                            allTriangles.push_back(
                                sf::Vertex{v_prev, fillColor});
                            allTriangles.push_back(
                                sf::Vertex{v_curr, fillColor});
                            v_prev = v_curr;
                        }
                    }
                } catch (...) {
                }
            };

            auto collectVertices = [&](Polygon_2 poly) {
                for (auto vit = poly.vertices_begin();
                     vit != poly.vertices_end(); ++vit) {
                    sf::CircleShape pt(3.f);
                    pt.setOrigin({3.f, 3.f});
                    pt.setPosition(GeoUtils::toSFML(*vit));
                    pt.setFillColor(sf::Color::White);
                    pt.setOutlineColor(sf::Color::Black);
                    pt.setOutlineThickness(1.f);
                    allVertices.push_back(pt);
                }
            };

            if (unifyPolygons) {
                auto unifiedPolys = identifier.getUnifiedThinWalls();
                for (const auto &pwh : unifiedPolys) {
                    triangulateAndAppend(pwh.outer_boundary(),
                                         sf::Color(220, 50, 50, 160));
                    collectVertices(pwh.outer_boundary());
                    for (auto hit = pwh.holes_begin(); hit != pwh.holes_end();
                         ++hit) {
                        Polygon_2 hole = *hit;
                        if (hole.orientation() == CGAL::CLOCKWISE)
                            hole.reverse_orientation();
                        triangulateAndAppend(hole, sf::Color(30, 30, 35, 255));
                        collectVertices(hole);
                    }
                }
            } else {

                auto chains = identifier.identifyThinWallChains();
                int colorIdx = 0;
                for (const auto &chain_polys : chains) {
                    sf::Color chainColor = palette[colorIdx % palette.size()];

                    for (const auto &poly : chain_polys) {
                        triangulateAndAppend(poly, chainColor);
                        collectVertices(poly);
                    }
                    colorIdx++;
                }
            }

            if (!allTriangles.empty()) {
                window.draw(allTriangles.data(), allTriangles.size(),
                            sf::PrimitiveType::Triangles);
            }

            for (const auto &pt : allVertices) {
                window.draw(pt);
            }

            if (!identifier.medialAxis.empty()) {
                std::vector<sf::Vertex> medLines;
                for (const auto &seg : identifier.medialAxis) {
                    auto pSrc = seg->opposite()->vertex()->point();
                    auto pTgt = seg->vertex()->point();
                    medLines.push_back(
                        sf::Vertex{GeoUtils::toSFML(pSrc), sf::Color::Green});
                    medLines.push_back(
                        sf::Vertex{GeoUtils::toSFML(pTgt), sf::Color::Green});
                }
                window.draw(medLines.data(), medLines.size(),
                            sf::PrimitiveType::Lines);
            }
        }
    }

    else {
        if (outerClosed && !identifier.medialAxis.empty()) {
            std::vector<sf::Vertex> medLines;
            for (const auto &seg : identifier.medialAxis) {
                auto pSrc = seg->opposite()->vertex()->point();
                auto pTgt = seg->vertex()->point();

                medLines.push_back(
                    sf::Vertex{GeoUtils::toSFML(pSrc), sf::Color(0, 130, 0)});
                medLines.push_back(
                    sf::Vertex{GeoUtils::toSFML(pTgt), sf::Color(0, 130, 0)});
            }
            window.draw(medLines.data(), medLines.size(),
                        sf::PrimitiveType::Lines);
        }

        for (const auto &res : identifier.debugThinWalls) {
            float r = static_cast<float>(res.localThickness / 2.0);
            sf::CircleShape c(r);
            c.setOrigin({r, r});
            c.setPosition(GeoUtils::toSFML(res.center));
            c.setFillColor(sf::Color(255, 165, 0, 100));
            c.setOutlineColor(sf::Color(255, 140, 0));
            c.setOutlineThickness(1);
            window.draw(c);
        }
    }

    window.draw(btnReset);
    window.draw(btnAddHole);
    window.draw(btnToggleView);
    window.draw(btnToggleArea);
    window.draw(sliderBg);
    window.draw(sliderHandle);
    window.draw(sliderPruneBg);
    window.draw(sliderPruneHandle);

    if (hasFont) {
        auto drawCenteredText = [&](const std::string &str, int size,
                                    sf::Color color,
                                    const sf::RectangleShape &btn) {
            sf::Text t(font, str, size);
            t.setFillColor(color);
            sf::FloatRect b = t.getLocalBounds();
            t.setOrigin(
                {b.position.x + b.size.x / 2.f, b.position.y + b.size.y / 2.f});
            t.setPosition({btn.getPosition().x + btn.getSize().x / 2.f,
                           btn.getPosition().y + btn.getSize().y / 2.f});
            window.draw(t);
        };

        drawCenteredText(outerClosed ? "Reset" : "...", 18, sf::Color::White,
                         btnReset);
        {
            bool inHoleMode = (drawingMode == DrawingMode::HOLE);
            btnAddHole.setFillColor(inHoleMode ? sf::Color::Yellow
                                               : sf::Color(70, 70, 70));
            window.draw(btnAddHole);
            sf::Color holeColor = !outerClosed ? sf::Color(100, 100, 60)
                                  : inHoleMode ? sf::Color::Black
                                               : sf::Color::Yellow;
            drawCenteredText(inHoleMode ? "Close Hole" : "Add Hole", 14,
                             holeColor, btnAddHole);
        }

        drawCenteredText("Skeleton", 16,
                         currentView == ViewMode::SKELETON ? sf::Color::Black
                                                           : sf::Color::Cyan,
                         btnToggleView);

        drawCenteredText("Thin Walls", 16,
                         currentView == ViewMode::AREA ? sf::Color::White
                                                       : sf::Color(255, 50, 50),
                         btnToggleArea);

        std::string label =
            "Max Thickness: " + std::to_string((int)identifier.maxThickness);
        sf::Text tLabel(font, label, 16);
        tLabel.setFillColor(sf::Color::White);
        tLabel.setPosition({362, 705});
        window.draw(tLabel);

        double effectiveThreshold =
            identifier.pruneFactor * identifier.maxThickness;
        std::ostringstream pruneStream;
        pruneStream << std::fixed << std::setprecision(1)
                    << identifier.pruneFactor << "x  (=" << effectiveThreshold
                    << ")";
        sf::Text tPrune(font, "Prune: " + pruneStream.str(), 13);
        tPrune.setFillColor(sf::Color(255, 160, 0));
        tPrune.setPosition({840, 30});
        window.draw(tPrune);
    }

    window.display();
}

void App::resetDrawing() {
    outerPoints.clear();
    completedHoles.clear();
    currentHolePoints.clear();
    outerClosed = false;
    drawingHole = false;
    drawingMode = DrawingMode::OUTER;
    currentView = ViewMode::ANALYSIS;
    identifier.setPolygon({});
}

bool App::loadFromFile(const std::string &name) {
    namespace fs = std::filesystem;
    fs::path resolved = name;
    if (!fs::exists(resolved)) {
        for (const char *ext : {"", ".txt"}) {
            fs::path p = fs::path(ASSETS_DIR) / "polygons" / (name + ext);
            if (fs::exists(p)) {
                resolved = p;
                break;
            }
        }
    }

    std::vector<sf::Vector2f> outer;
    std::vector<std::vector<sf::Vector2f>> holes;
    if (!GeoUtils::loadPolygonFromFile(resolved.string(), outer, holes))
        return false;

    resetDrawing();
    outerPoints = std::move(outer);
    completedHoles = std::move(holes);
    outerClosed = true;
    loadedFileName = resolved.string();
    window.setTitle("thin walls - " + resolved.filename().string());
    rebuildAndAnalyze();
    return true;
}
