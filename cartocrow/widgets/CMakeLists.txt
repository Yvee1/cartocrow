set(SOURCES
    double_slider.cpp
)

set(HEADERS
    double_slider.h
)

add_library(widgets ${SOURCES})

target_link_libraries(
    widgets
    PUBLIC
    Qt5::Widgets
)

install(TARGETS widgets DESTINATION ${INSTALL_BINARY_DIR})

cartocrow_install_module(widgets)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/widgets)
