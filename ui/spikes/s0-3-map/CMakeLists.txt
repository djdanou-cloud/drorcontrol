# Spike S0-3 (T0-07): MapLibre Native rendering through Qt RHI (Metal on macOS) — frame timing.
# Standalone: needs Qt >= 6.5 and QMapLibre built from the pinned commit (see build.sh).
cmake_minimum_required(VERSION 3.21)
project(SpikeS03Map VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS Quick)
qt_standard_project_setup(REQUIRES 6.5)
find_package(QMapLibre COMPONENTS Core Quick REQUIRED)

qt_add_executable(spike-s0-3-map main.cpp)
qt_add_qml_module(spike-s0-3-map
    URI S03
    VERSION 1.0
    QML_FILES Main.qml
)
target_link_libraries(spike-s0-3-map PRIVATE Qt::Quick QMapLibre::Core)
qmaplibre_quick_setup_plugins(spike-s0-3-map)
# For a shared QMapLibre the helper only tells Qt's tooling where the MapLibre QML module is;
# hand the same directory to the running engine.
get_target_property(S03_MAPLIBRE_QML spike-s0-3-map QT_QML_IMPORT_PATH)
if(S03_MAPLIBRE_QML)
    target_compile_definitions(spike-s0-3-map PRIVATE S03_MAPLIBRE_QML="${S03_MAPLIBRE_QML}")
endif()
