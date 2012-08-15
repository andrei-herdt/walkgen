string(TOLOWER ${CURRENT_QPSOLVER} lsolver)
add_definitions(" -DCURRENT_QPSOLVER=QPSOLVER_${CURRENT_QPSOLVER}")

create_test(test-humanoid-${lsolver}
  SRC test-humanoid.cpp ../../src/common/mpc-debug.cpp
  INTDEPS mpc-walkgen timer)