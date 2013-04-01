##	Filename:  unittests/c++/Makefile
##	Author:    Andrei Herdt

# select your operating system here!
include make_dependencies.mk


##
##	flags
##

IFLAGS      =  -I. \
               -I${I_DIR} \
			   -I${EIGEN_DIR}

TEST_EXES = 	walk${EXE} \
		test_zero_map${EXE} \
		test_const_map${EXE} \
		measure_time${EXE} \
		dynamics_test${EXE}

##
##	targets
##
all: ${TEST_EXES}

	
walk${EXE}: walk.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${QLD_LIB} ${BLAS_LIB} ${LAPACK_LIB}

test_zero_map${EXE}: test_zero_map.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${QLD_LIB} ${BLAS_LIB} ${LAPACK_LIB}

test_const_map${EXE}: test_const_map.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${QLD_LIB} ${BLAS_LIB} ${LAPACK_LIB}

measure_time${EXE}: measure_time.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${QLD_LIB} ${BLAS_LIB} ${LAPACK_LIB}

dynamics_test${EXE}: dynamics_test.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${QLD_LIB} ${BLAS_LIB} ${LAPACK_LIB}



clean: 
	${RM} -f *.${OBJEXT} ${TEST_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

##
##	end of file
##
