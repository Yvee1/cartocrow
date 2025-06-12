set(SOURCES
    treemap_demo.cpp
)

set(HEADERS
    treemap_demo.h
)

add_executable(treemap_demo ${SOURCES})

target_link_libraries(
        treemap_demo
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        treemap
        CGAL::CGAL
        Qt5::Widgets
)

install(TARGETS treemap_demo DESTINATION ${INSTALL_BINARY_DIR})

add_subdirectory(treemap_interactive)