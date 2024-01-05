set(SOURCES
    medial_axis_demo.cpp
        medial_axis.cpp
        medial_axis.h
)

add_executable(medial_axis_demo ${SOURCES})

target_link_libraries(
    medial_axis_demo
    PRIVATE
    ${COMMON_CLA_TARGET}
    core
    renderer
    CGAL::CGAL
    Qt5::Widgets
    glog::glog
)

install(TARGETS medial_axis_demo DESTINATION ${INSTALL_BINARY_DIR})
