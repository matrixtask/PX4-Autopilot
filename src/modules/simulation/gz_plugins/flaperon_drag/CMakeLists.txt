# Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause.
# Drag of deflected flaperons/spoilers (Mk-7 models, plugin filename "FlaperonDrag").
project(FlaperonDrag)

add_library(${PROJECT_NAME} SHARED FlaperonDrag.cpp)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
