set(SOURCES
    cgal_tutorial.cpp
)

add_executable(cgal_tutorial ${SOURCES})

target_link_libraries(
    cgal_tutorial
    PRIVATE
    ${COMMON_CLA_TARGET}
    core
    renderer
    CGAL::CGAL
    Qt5::Widgets
    glog::glog
)

install(TARGETS cgal_tutorial DESTINATION ${INSTALL_BINARY_DIR})
