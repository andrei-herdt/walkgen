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

TEST_EXES = walk${EXE} \
			pid-mode${EXE}


##
##	targets
##
all: ${TEST_EXES}

	
walk${EXE}: walk.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${BLAS_LIB}

pid-mode${EXE}: pid-mode.${OBJEXT} ${SRC_DIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${SRC_DIR}libmpc-walkgen.${LIBEXT} ${THREAD_LIB} ${QPOASES_LIB} ${BLAS_LIB}
	
clean: 
	${RM} -f *.${OBJEXT} ${TEST_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

##
##	end of file
##
