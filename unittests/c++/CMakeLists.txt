include_directories(BEFORE ${PROJECT_SOURCE_DIR}/src)

# Add static libraries to gather some code used in the unit tests
# and avoid compiling the same files over and over

# the humanoid library holds the "QP-solving subset" of mpc-walkgen,
set(humanoid_SRC
  ../src/qp-solver.cpp
  ../src/qp-matrix.cpp
  ../src/qp-vector.cpp
  ../src/types.cpp
  ../src/sharedtypes.cpp
  ../src/walkgen-abstract.cpp
  ../src/lssol-parser.cpp
  ../src/qpoases-parser.cpp)

add_library(humanoid STATIC ${humanoid_SRC})

# the timer library holds some timing and benchmark tools.
add_library(timer STATIC
  ../src/mpc-debug.cpp
  ../src/gettimeofday.cpp)

create_test(test-walk
  SRC test-walk.cpp ../src/mpc-debug.cpp
  INTDEPS mpc-walkgen timer)