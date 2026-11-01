############################################################################
#
#   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
#   BSD-3-Clause, same terms as PX4.
#
############################################################################

# Header-only ground-plane fit (PlaneFit.hpp) and landing profile / site search (LandingProfile.hpp).
px4_add_unit_gtest(SRC PlaneFitTest.cpp)
px4_add_unit_gtest(SRC LandingProfileTest.cpp)
