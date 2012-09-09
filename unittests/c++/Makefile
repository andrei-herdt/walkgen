##	Filename:  unittests/c++/Makefile
##	Author:    Andrei Herdt (Joachim Ferreau)

# select your operating system here!
include ../../src/make_linux.mk
# include ../../src/make_windows.mk
# include ../../src/make_vxworks.mk
# include ../../src/make_vxworks_intel.mk



##
##	flags
##
IDIR				=  ../../include/
SRCDIR				=  ../../src/
EIGENDIR			=	../../../eigen-3.1.1/
QPOASES_SRCDIR		=	../../../qpoases-3.0/src/
QPOASES_LIB			=	${QPOASES_SRCDIR}libqpOASES.a
BLAS_LIB			=	../../../GotoBLAS2/libgoto2.a

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
