##
##	Filename:  src/Makefile
##	Author:    Andrei Herdt


SRCDIR = .

# select your operating system here!
include make_dependencies.mk


##
##	flags
##
IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${DIR_EIGEN} \
			   -I${QPOASES_IDIR}

MPC_WALKGEN_OBJECTS = \
	walkgen.${OBJEXT} \
	types.${OBJEXT} \
	tools.${OBJEXT} \
	state-fsm.${OBJEXT} \
	rigid-body-system.${OBJEXT} \
	rigid-body.${OBJEXT} \
	qp-vector.${OBJEXT} \
	qp-solver.${OBJEXT} \
	heuristic-preview.${OBJEXT} \
	qpoases-parser.${OBJEXT} \
	qp-matrix.${OBJEXT} \
	qp-builder.${OBJEXT} \
	orientations-preview.${OBJEXT} \
	lssol-parser.${OBJEXT} \
	interpolation.${OBJEXT} \
	foot-body.${OBJEXT} \
	com-body.${OBJEXT} \
	realclock.${OBJEXT} \
	dynamics-builder.${OBJEXT} 
	

all: libmpc-walkgen.${LIBEXT}


libmpc-walkgen.${LIBEXT}: ${MPC_WALKGEN_OBJECTS} 
	${AR} r $@ $?
	
	
##
##	targets
##
walkgen.${OBJEXT}:  walkgen.cpp \
	${IDIR}/mpc-walkgen/walkgen.h \
	${IDIR}/mpc-walkgen/orientations-preview.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/state-fsm.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/realclock.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} walkgen.cpp

types.${OBJEXT}: types.cpp \
	${IDIR}/mpc-walkgen/types.h \
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
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} state-fsm.cpp

rigid-body-system.${OBJEXT}: rigid-body-system.cpp \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} rigid-body-system.cpp
	
rigid-body.${OBJEXT}: rigid-body.cpp \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h \
	${IDIR}/mpc-walkgen/dynamics-builder.h 
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
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-solver.cpp
	
heuristic-preview.${OBJEXT}: heuristic-preview.cpp \
	${IDIR}/mpc-walkgen/heuristic-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/state-fsm.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} heuristic-preview.cpp	
	
qpoases-parser.${OBJEXT}: qpoases-parser.cpp \
	${IDIR}/mpc-walkgen/qpoases-parser.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${QPOASES_IDIR}/qpOASES.hpp \
	${QPOASES_IDIR}/qpOASES/SQProblem.hpp \
	${QPOASES_IDIR}/qpOASES/QProblem.hpp \
	${QPOASES_IDIR}/qpOASES/QProblemB.hpp \
	${QPOASES_IDIR}/qpOASES/Bounds.hpp \
	${QPOASES_IDIR}/qpOASES/Constraints.hpp \
	${QPOASES_IDIR}/qpOASES/SubjectTo.hpp \
	${QPOASES_IDIR}/qpOASES/Indexlist.hpp \
	${QPOASES_IDIR}/qpOASES/Utils.hpp \
	${QPOASES_IDIR}/qpOASES/Constants.hpp \
	${QPOASES_IDIR}/qpOASES/Types.hpp \
	${QPOASES_IDIR}/qpOASES/Options.hpp \
	${QPOASES_IDIR}/qpOASES/Matrices.hpp \
	${QPOASES_IDIR}/qpOASES/MessageHandling.hpp
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qpoases-parser.cpp	
	
qp-matrix.${OBJEXT}: qp-matrix.cpp \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-matrix.cpp
	
qp-builder.${OBJEXT}: qp-builder.cpp \
	${IDIR}/mpc-walkgen/qp-builder.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/qp-solver.h \
	${IDIR}/mpc-walkgen/qp-matrix.h \
	${IDIR}/mpc-walkgen/qp-vector.h \
	${IDIR}/mpc-walkgen/heuristic-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h \
	${IDIR}/mpc-walkgen/state-fsm.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} qp-builder.cpp

orientations-preview.${OBJEXT}: orientations-preview.cpp \
	${IDIR}/mpc-walkgen/orientations-preview.h \
	${IDIR}/mpc-walkgen/rigid-body-system.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h  \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/rigid-body-system-inl.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} orientations-preview.cpp

interpolation.${OBJEXT}: interpolation.cpp \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/types.h \
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
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} lssol-parser.cpp	

foot-body.${OBJEXT}: foot-body.cpp \
	${IDIR}/mpc-walkgen/foot-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h \
	${IDIR}/mpc-walkgen/dynamics-builder.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} foot-body.cpp
	
com-body.${OBJEXT}: com-body.cpp \
	${IDIR}/mpc-walkgen/com-body.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h \
	${IDIR}/mpc-walkgen/rigid-body.h \
	${IDIR}/mpc-walkgen/interpolation.h \
	${IDIR}/mpc-walkgen/tools.h \
	${IDIR}/mpc-walkgen/tools-inl.h \
	${IDIR}/mpc-walkgen/dynamics-builder.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} com-body.cpp

realclock.${OBJEXT}: realclock.cpp \
	${IDIR}/mpc-walkgen/realclock.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} realclock.cpp
	
dynamics-builder.${OBJEXT}: dynamics-builder.cpp \
	${IDIR}/mpc-walkgen/dynamics-builder.h \
	${IDIR}/mpc-walkgen/types.h \
	${IDIR}/mpc-walkgen/api.h 
	@echo "Creating"  $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} dynamics-builder.cpp

clean:
	${RM} -f *.${OBJEXT} *.${LIBEXT}

clobber: clean
	
shared:	${MPC_WALKGEN_OBJECTS}
	${CPP} -shared -Wl,-soname,libmpc-walkgen.so -o libmpc-walkgen.so ${MPC_WALKGEN_OBJECTS} -lm

##
##	end of file
##
