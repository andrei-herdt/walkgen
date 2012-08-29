##
##	Filename:  src/Makefile
##	Author:    Andrei Herdt


SRCDIR = .

# select your operating system here!
# include make_linux.mk
# include make_windows.mk
 include make_vxworks.mk
#include make_vxworks_intel.mk


##
##	flags
##
IDIR        =  ../include/
DIR_EIGEN   =	../../eigen-3.1.1/
IDIR3		=	../../qpOASES-3.0/include/
SRC_QPOASES =  ../../qpOASES-3.0/src/

IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${DIR_EIGEN} \
			   -I${IDIR3}

MPC_WALKGEN_OBJECTS = \
	walkgen-abstract.${OBJEXT} \
	walkgen.${OBJEXT} \
	types.${OBJEXT} \
	tools.${OBJEXT} \
	state-fsm.${OBJEXT} \
	sharedtypes.${OBJEXT} \
	rigid-body-system.${OBJEXT} \
	rigid-body.${OBJEXT} \
	qp-vector.${OBJEXT} \
	qp-solver.${OBJEXT} \
	qp-preview.${OBJEXT} \
	qpoases-parser.${OBJEXT} \
	qp-matrix.${OBJEXT} \
	qp-generator.${OBJEXT} \
	orientations-preview.${OBJEXT} \
	lssol-parser.${OBJEXT} \
	interpolation.${OBJEXT} \
	foot-body.${OBJEXT} \
	com-body.${OBJEXT} \
	stopwatch.${OBJEXT} 
	
QPOASES_OBJECTS = \
	${SRC_QPOASES}/SQProblem.${OBJEXT} \
	${SRC_QPOASES}/QProblem.${OBJEXT} \
	${SRC_QPOASES}/QProblemB.${OBJEXT} \
	${SRC_QPOASES}/Bounds.${OBJEXT} \
	${SRC_QPOASES}/Constraints.${OBJEXT} \
	${SRC_QPOASES}/SubjectTo.${OBJEXT} \
	${SRC_QPOASES}/Indexlist.${OBJEXT} \
	${SRC_QPOASES}/Utils.${OBJEXT} \
	${SRC_QPOASES}/Options.${OBJEXT} \
	${SRC_QPOASES}/Matrices.${OBJEXT} \
	${SRC_QPOASES}/MessageHandling.${OBJEXT}
	

all: libmpc-walkgen.${LIBEXT}


libmpc-walkgen.${LIBEXT}: ${MPC_WALKGEN_OBJECTS} \
	${QPOASES_OBJECTS}
	${AR} r $@ $?

##
##	targets
##
walkgen-abstract.${OBJEXT}: walkgen-abstract.cpp \
	${IDIR}/mpc-walkgen/walkgen-abstract.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} walkgen-abstract.cpp


walkgen.${OBJEXT}:  walkgen.cpp \
	${IDIR}/mpc-walkgen/walkgen.h \
	${IDIR}/mpc-walkgen/walkgen-abstract.h \
	${IDIR}/mpc-walkgen/orientations-preview.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/state-fsm.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/stopwatch.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} walkgen.cpp

types.${OBJEXT}: types.cpp \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} types.cpp

tools.${OBJEXT}: tools.cpp \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} tools.cpp
	
state-fsm.${OBJEXT}: state-fsm.cpp \
	${IDIR}/mpc-walkgen/state-fsm.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} state-fsm.cpp

sharedtypes.${OBJEXT}: sharedtypes.cpp \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} sharedtypes.cpp
	
rigid-body-system.${OBJEXT}: rigid-body-system.cpp \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} rigid-body-system.cpp
	
rigid-body.${OBJEXT}: rigid-body.cpp \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} rigid-body.cpp
	
qp-vector.${OBJEXT}: qp-vector.cpp \
	${IDIR}/mpc-walkgen/qp-vector.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-vector.cpp
	
qp-solver.${OBJEXT}: qp-solver.cpp \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-solver.cpp
	
qp-preview.${OBJEXT}: qp-preview.cpp \
	${IDIR}/mpc-walkgen/qp-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/state-fsm.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-preview.cpp	
	
qpoases-parser.${OBJEXT}: qpoases-parser.cpp \
	${IDIR}/mpc-walkgen/qpoases-parser.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR3}/qpOASES.hpp \
	${IDIR3}/qpOASES/SQProblem.hpp \
	${IDIR3}/qpOASES/QProblem.hpp \
	${IDIR3}/qpOASES/QProblemB.hpp \
	${IDIR3}/qpOASES/Bounds.hpp \
	${IDIR3}/qpOASES/Constraints.hpp \
	${IDIR3}/qpOASES/SubjectTo.hpp \
	${IDIR3}/qpOASES/Indexlist.hpp \
	${IDIR3}/qpOASES/Utils.hpp \
	${IDIR3}/qpOASES/Constants.hpp \
	${IDIR3}/qpOASES/Types.hpp \
	${IDIR3}/qpOASES/Options.hpp \
	${IDIR3}/qpOASES/Matrices.hpp \
	${IDIR3}/qpOASES/MessageHandling.hpp
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qpoases-parser.cpp	
	
qp-matrix.${OBJEXT}: qp-matrix.cpp \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-matrix.cpp
	
qp-generator.${OBJEXT}: qp-generator.cpp \
	${IDIR}/mpc-walkgen/qp-generator.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/qp-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/state-fsm.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-generator.cpp

orientations-preview.${OBJEXT}: orientations-preview.cpp \
	${IDIR}/mpc-walkgen/orientations-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} orientations-preview.cpp

interpolation.${OBJEXT}: interpolation.cpp \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} interpolation.cpp
	
lssol-parser.${OBJEXT}: lssol-parser.cpp \
	${IDIR}/mpc-walkgen/lssol-parser.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} lssol-parser.cpp	

foot-body.${OBJEXT}: foot-body.cpp \
	${IDIR}/mpc-walkgen/foot-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} foot-body.cpp
	
com-body.${OBJEXT}: com-body.cpp \
	${IDIR}/mpc-walkgen/com-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/sharedtypes.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} com-body.cpp

stopwatch.${OBJEXT}: stopwatch.cpp \
	${IDIR}/mpc-walkgen/stopwatch.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} stopwatch.cpp

clean:
	${RM} -f *.${OBJEXT} *.${LIBEXT}

clobber: clean

#%.${OBJEXT}: %.cpp
	#@echo "Creating" $@
	#${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

	
shared:	${MPC_WALKGEN_OBJECTS}
	${CPP} -shared -Wl,-soname,libmpc-walkgen.so -o libmpc-walkgen.so ${MPC_WALKGEN_OBJECTS} -lm

##
##	end of file
##
