set(SOURCES
    render_state_demo.cpp
    ../colors/colors.cpp
)

add_executable(render_state_demo ${SOURCES})

target_link_libraries(
        render_state_demo
        PRIVATE
        ${COMMON_CLA_TARGET}
        core
        renderer
        kinetic_kelp
        Qt5::Widgets
)

install(TARGETS render_state_demo DESTINATION ${INSTALL_BINARY_DIR})
