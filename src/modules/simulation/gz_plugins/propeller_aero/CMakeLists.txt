# Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause.
# Advance-ratio thrust lapse and windmilling drag for a MulticopterMotorModel propeller (Mk-7 pusher).
project(PropellerAero)
add_library(${PROJECT_NAME} SHARED PropellerAero.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${GZ_PLUGIN_TARGET} PUBLIC ${GZ_SIM_TARGET})
if (NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
