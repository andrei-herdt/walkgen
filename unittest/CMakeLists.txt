include_directories(BEFORE ${PROJECT_SOURCE_DIR}/src)

# Add static libraries to gather some code used in the unit tests
# and avoid compiling the same files over and over

# the qpsolverhumanoid library holds the QP-solving subset of mpc-walkgen,
# with the types defined for the "humanoid" problem.
set(qpsolverhumanoid_SRC
  ../src/common/qp-solver.cpp
  ../src/common/qp-matrix.cpp
  ../src/common/qp-vector.cpp
  ../src/humanoid/types.cpp
  ../src/humanoid/sharedpgtypes.cpp
  ../src/humanoid/walkgen-abstract.cpp)
if(WITH_LSSOL)
  list(APPEND qpsolverhumanoid_SRC ../src/common/qp-solvers/lssol-parser.cpp)
endif()
if(WITH_QPOASES)
  list(APPEND qpsolverhumanoid_SRC ../src/common/qp-solvers/qpoases-parser.cpp)
endif()
add_library(qpsolverhumanoid STATIC ${qpsolverhumanoid_SRC})

# the timer library holds some timing and benchmark tools.
add_library(timer STATIC
  ../src/common/mpc-debug.cpp
  ../src/common/gettimeofday.cpp)

# the "with-solver" subfolder is included once per supported solver. The tests
# defined within it should use the "CURRENT_QPSOLVER" macro to use the
# correpsonding solver.
# For each inclusion of the folder, a different binary folder is used. It acts
# as a namespace for CMake. This makes it possible to use different values
# for "CURRENT_QPSOLVER" when calling add_definitions(), without conflicts.
if(WITH_LSSOL)
  set(CURRENT_QPSOLVER "LSSOL")
  add_subdirectory(use-solver use-lssol)
endif()
if(WITH_QPOASES)
   set(CURRENT_QPSOLVER "QPOASES")
  add_subdirectory(use-solver use-qpoases)
endif()
