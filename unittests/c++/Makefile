##	Filename:  unittests/c++/Makefile
##	Author:    Andrei Herdt (Joachim Ferreau)

# select your operating system here!
#include ../../src/make_linux.mk
#include ../../src/make_windows.mk
include ../../src/make_vxworks.mk
#include ../../src/make_vxworks_intel.mk



##
##	flags
##
IDIR        =  ../../include/
SRCDIR      =  ../../src/
DIR_EIGEN		=	../../../eigen-3.1.1/
QPOASES_LIB =	../../../qpOASES-3.0/LIB/VXWORKS/libqpOASES.a

IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${DIR_EIGEN}

TEST_EXES = \
	walk${EXE}


##
##	targets
##
all: ${TEST_EXES}

walk${EXE}: walk.${OBJEXT} ${SRCDIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPC_WALKGEN_LIB} ${QPOASES_LIB}

clean:
	${RM} -f *.${OBJEXT} ${TEST_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##
