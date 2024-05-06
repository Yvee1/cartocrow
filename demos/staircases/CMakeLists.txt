set(SOURCES
    staircases_demo.cpp
    staircase.cpp
    staircase.h
    types.h
    brackets.cpp
    brackets.h
    move.cpp
    move.h
    greedy.cpp
    greedy.h
    optimal.cpp
    optimal.h
)

add_executable(staircases_demo ${SOURCES})

target_link_libraries(
        staircases_demo
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        CGAL::CGAL
        Qt5::Widgets
        glog::glog
)

install(TARGETS staircases_demo DESTINATION ${INSTALL_BINARY_DIR})