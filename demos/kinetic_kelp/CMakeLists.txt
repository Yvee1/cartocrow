set(SOURCES
    kinetic_kelp_demo.cpp
    colors/colors.cpp
    time_control_widget.cpp
    time_control_widget.h
)

add_executable(kinetic_kelp_demo ${SOURCES})

target_link_libraries(
    kinetic_kelp_demo
    PRIVATE
    ${COMMON_CLA_TARGET}
    core
    renderer
    kinetic_kelp
    CGAL::CGAL
    Qt5::Widgets
)

add_subdirectory(render_state_demo)

install(TARGETS kinetic_kelp_demo DESTINATION ${INSTALL_BINARY_DIR})
