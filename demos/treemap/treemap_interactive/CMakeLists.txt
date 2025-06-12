set(SOURCES
    treemap_interactive.cpp
)

set(HEADERS
    treemap_interactive.h
)

add_executable(treemap_interactive ${SOURCES})

target_link_libraries(
        treemap_interactive
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        treemap
        CGAL::CGAL
        Qt5::Widgets
)

install(TARGETS treemap_interactive DESTINATION ${INSTALL_BINARY_DIR})