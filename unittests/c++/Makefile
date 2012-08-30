##	Filename:  unittests/c++/Makefile
##	Author:    Andrei Herdt (Joachim Ferreau)

# select your operating system here!
# include ../../src/make_linux.mk
# include ../../src/make_windows.mk
# include ../../src/make_vxworks.mk
include ../../src/make_vxworks_intel.mk



##
##	flags
##
IDIR				=  ../../include/
SRCDIR				=  ../../src/
MAINDIR				=	/home/f_legs/rtw_models/herdt/mpc/
TESTDIR				=	/home/f_legs/rtw_models/herdt/mpc/unittests/c++/
SRCDIR				=	/home/f_legs/rtw_models/herdt/mpc/src/
EIGENDIR			=	/home/f_legs/rtw_models/herdt/eigen-3.1.1/
QPOASES_DIR			=	/home/f_legs/rtw_models/herdt/qpOASES-3.0/
QPOASES_SRCDIR		=	/home/f_legs/rtw_models/herdt/qpOASES-3.0/src/
QPOASES_LIB			=	${QPOASES_SRCDIR}libqpOASES.a

IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${EIGENDIR}

TEST_EXES = walk${EXE}


##
##	targets
##
all: ${TEST_EXES}
	
##@  cd ${QPOASES_SRCDIR}		&& ${MAKE} && cd ${TESTDIR} \
##&& cd ${SRCDIR}				&& ${MAKE} && cd ${TESTDIR} \
##&& ${TEST_EXES}
	
walk${EXE}: walk.${OBJEXT} ${SRCDIR}libmpc-walkgen.${LIBEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPC_WALKGEN_LIB} ${QPOASES_LIB}

clean: ${RM} -f *.${OBJEXT} ${TEST_EXES}
	
##@  cd ${QPOASES_SRCDIR}    && ${RM} -f *.${OBJEXT} ${TEST_EXES} && cd ${TESTDIR} \
##&& cd ${SRCDIR}          && ${RM} -f *.${OBJEXT} ${TEST_EXES} && cd ${TESTDIR}
	

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

##
##	end of file
##
