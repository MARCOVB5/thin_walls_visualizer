#include "App.hpp"
#include <string>

int main(int argc, char *argv[]) {
    std::string polyName = "";

    if (argc >= 2) {
        polyName = argv[1];
    }

    App app(polyName);
    app.run();
    return 0;
}
