%%	Filename:  interfaces/matlab/simulink/make.m
%%	Author:    Andrei Herdt
%%	Version:
%%	Date:      2012
%%
%% consistency check
if ( exist( [pwd, '/make.m'],'file' ) == 0 )
    disp( 'ERROR: Run this make script directly within the directory' );
    disp( '       <mpc-walkgen>/interfaces/matlab/simulink, please.' );
    return;
end

%% Set dependencies
make_dependencies;

%% Set compile options
make_release;
%make_debug;

IFLAGS  = ['-I. -I', MPC_WALKGEN_PATH, 'include',' -I',EIGEN_PATH,' -I',QPOASES_PATH,'include/',' -I',QLD_PATH,' ' ];
CPPFLAGS  = [IFLAGS, CFLAGS];

MPC_WALKGEN_OBJECTS =	[	MPC_WALKGEN_PATH, 'src/com-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/foot-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/interpolation.cpp ',...
    MPC_WALKGEN_PATH, 'src/lssol-parser.cpp ',...
    MPC_WALKGEN_PATH, 'src/realclock.cpp ',...
    MPC_WALKGEN_PATH, 'src/orientations-preview.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-builder.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-matrix.cpp ',...
    MPC_WALKGEN_PATH, 'src/heuristic-preview.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-solver.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-vector.cpp ',...
    MPC_WALKGEN_PATH, 'src/qpoases-parser.cpp ',...
    MPC_WALKGEN_PATH, 'src/qld-parser.cpp ',...
    MPC_WALKGEN_PATH, 'src/rigid-body-system.cpp ',...
    MPC_WALKGEN_PATH, 'src/rigid-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/state-fsm.cpp ',...
    MPC_WALKGEN_PATH, 'src/tools.cpp ',...
    MPC_WALKGEN_PATH, 'src/types.cpp ',...
    MPC_WALKGEN_PATH, 'src/dynamics-builder.cpp ',...
    MPC_WALKGEN_PATH, 'src/debug.cpp ',...
    MPC_WALKGEN_PATH, 'src/walkgen.cpp ', ' ' ];

QPOASES_OBJECTS =	[	QPOASES_PATH, 'src/SQProblem.cpp ',...
    QPOASES_PATH, 'src/QProblem.cpp ',...
    QPOASES_PATH, 'src/QProblemB.cpp ',...
    QPOASES_PATH, 'src/Bounds.cpp ',...
    QPOASES_PATH, 'src/Constraints.cpp ',...
    QPOASES_PATH, 'src/SubjectTo.cpp ',...
    QPOASES_PATH, 'src/Indexlist.cpp ',...
    QPOASES_PATH, 'src/Flipper.cpp ',...
    QPOASES_PATH, 'src/Utils.cpp ',...
    QPOASES_PATH, 'src/Options.cpp ',...
    QPOASES_PATH, 'src/Matrices.cpp ',...
    QPOASES_PATH, 'src/LAPACKReplacement.cpp ',...
    QPOASES_PATH, 'src/BLASReplacement.cpp ',...
    QPOASES_PATH, 'src/MessageHandling.cpp ', ' ' ];
    %QPOASES_PATH, 'src/realtimeclock.cpp ',...
    
QLD_OBJECTS =	[	QLD_PATH, 'qld.cpp ', ' ' ];

NAME = 'walkgen_sfun';
%% Compile
eval(['mex -v -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ', QLD_OBJECTS, MPC_WALKGEN_OBJECTS, QPOASES_OBJECTS]]);
disp([NAME, '.', eval('mexext'), ' successfully created!']);

%% Set path
path(path, pwd);

%% Clear
clear EIGEN_PATH QPOASES_PATH QPOASES_LIB_PATH MPC_WALKGEN_PATH MPC_WALKGEN_LIBRARY_PATH IFLAGS CPPFLAGS CFLAGS MPC_WALKGEN_OBJECTS DEBUGFLAGS NAME QPOASES_OBJECTS QLD_PATH QLD_OBJECTS