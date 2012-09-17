##	Filename:  unittests/c++/Makefile
##	Author:    Andrei Herdt (Joachim Ferreau)

# select your operating system here!
include make_dependencies.mk
include ../../src/make_linux.mk
# include ../../src/make_windows.mk
# include ../../src/make_vxworks.mk
# include ../../src/make_vxworks_intel.mk



##
##	flags
##

IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${EIGENDIR}

TEST_EXES = walk${EXE}


##
##	targets
##
all: ${TEST_EXES}

	
walk${EXE}: walk.${OBJEXT} ${SRCDIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPC_WALKGEN_LIB} ${THREAD_LIB} ${QPOASES_LIB} ${BLAS_LIB}

clean: 
	${RM} -f *.${OBJEXT} ${TEST_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

##
##	end of file
##
