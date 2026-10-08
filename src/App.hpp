#ifndef APP_HPP
#define APP_HPP

#include "ThinWallIdentifier.hpp"
#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

enum class ViewMode { ANALYSIS, SKELETON, AREA };
enum class DrawingMode { OUTER, HOLE };

class App {
  private:
    sf::RenderWindow window;
    ThinWallIdentifier identifier;
    std::vector<sf::Vector2f> outerPoints;
    std::vector<std::vector<sf::Vector2f>> completedHoles;
    std::vector<sf::Vector2f> currentHolePoints;
    bool outerClosed;
    bool drawingHole;
    bool unifyPolygons;
    DrawingMode drawingMode;
    ViewMode currentView;
    sf::Font font;
    bool hasFont;
    sf::RectangleShape btnReset;
    sf::RectangleShape btnToggleView;
    sf::RectangleShape btnToggleArea;
    sf::RectangleShape btnAddHole;
    sf::RectangleShape sliderBg;
    sf::RectangleShape sliderHandle;
    bool draggingSlider;
    sf::RectangleShape sliderPruneBg;
    sf::RectangleShape sliderPruneHandle;
    bool draggingPruneSlider;
    void handleEvents();
    void render();
    void updateSliderPos();
    void updateThicknessFromSlider(float x);
    void updatePruneSliderPos();
    void updatePruneFromSlider(float x);
    void rebuildAndAnalyze();
    void drawPolygonOutline(const std::vector<sf::Vector2f> &pts, bool closed,
                            sf::Color color, bool dimmed = false);
    std::string loadedFileName;
    void resetDrawing();
    bool loadFromFile(const std::string &path);

  public:
    explicit App(const std::string &polyName = "");
    void run();
};

#endif
