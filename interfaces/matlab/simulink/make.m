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

%% Set flags
MPC_WALKGEN_PATH = '../../../';
%MPC_WALKGEN_LIBRARY_PATH = [MPC_WALKGEN_PATH,'build-new/Debug/'];
%MPC_WALKGEN_LIBRARY_PATH = [MPC_WALKGEN_PATH,'build-new/Release/'];
EIGEN_PATH = [MPC_WALKGEN_PATH,'deps/eigen/'];
QPOASES_PATH = [MPC_WALKGEN_PATH,'deps/qpOASES-3.0beta_git/include/'];
QPOASES_LIB_PATH = [MPC_WALKGEN_PATH,'deps/qpOASES-3.0beta_git/src/'];

%IFLAGS  = ['-I. -I',MPC_WALKGEN_PATH,'include',' -I',EIGEN_PATH,' -I',QPOASES_PATH,' -L',MPC_WALKGEN_LIBRARY_PATH,' ' ];
IFLAGS  = ['-I. -I',MPC_WALKGEN_PATH,'include',' -I',EIGEN_PATH,' -I',QPOASES_PATH,' ' ];

if ( ispc == 0 )
    %CPPFLAGS  = [ IFLAGS, '-D__cpluplus -D__MATLAB__ -O -DLINUX', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
else
    %CPPFLAGS  = [IFLAGS, '-v', ' -DWIN32',' -D__NO_COPYRIGHT__',' -D__SUPPRESSANYOUTPUT__', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
    CPPFLAGS  = [IFLAGS, ' -Wall', ' -pedantic', ' -Wshadow', ' -Wfloat-equal', ' -O3', ' -finline-functions', ' -D__VXWORKS__', ' ' ];
    %CPPFLAGS  = [IFLAGS, ' ' ];
     

end

%MPC_WALKGEN_OBJECTS = [MPC_WALKGEN_LIBRARY_PATH,'mpc-walkgen.lib ', ' '];
MPC_WALKGEN_OBJECTS =	[	MPC_WALKGEN_PATH, 'src/com-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/foot-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/gettimeofday.cpp ',...
    MPC_WALKGEN_PATH, 'src/interpolation.cpp ',...
    MPC_WALKGEN_PATH, 'src/lssol-parser.cpp ',...
    MPC_WALKGEN_PATH, 'src/mpc-debug.cpp ',...
    MPC_WALKGEN_PATH, 'src/orientations-preview.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-generator.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-matrix.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-preview.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-solver.cpp ',...
    MPC_WALKGEN_PATH, 'src/qp-vector.cpp ',...
    MPC_WALKGEN_PATH, 'src/qpoases-parser.cpp ',...
    MPC_WALKGEN_PATH, 'src/rigid-body-system.cpp ',...
    MPC_WALKGEN_PATH, 'src/rigid-body.cpp ',...
    MPC_WALKGEN_PATH, 'src/sharedtypes.cpp ',...
    MPC_WALKGEN_PATH, 'src/state-fsm.cpp ',...
    MPC_WALKGEN_PATH, 'src/tools.cpp ',...
    MPC_WALKGEN_PATH, 'src/types.cpp ',...
    MPC_WALKGEN_PATH, 'src/walkgen-abstract.cpp ',...
    MPC_WALKGEN_PATH, 'src/walkgen.cpp ', ' ' ];

QPOASES_OBJECTS = [	QPOASES_LIB_PATH, 'BLASReplacement.lib ',...
    QPOASES_LIB_PATH, 'LAPACKReplacement.lib ',...
    QPOASES_LIB_PATH, 'libqpOASES.lib ',...
    QPOASES_LIB_PATH, 'libqpOASESextras.lib ',' ' ];

% DEBUGFLAGS = ['-D_DEBUG -g',' '];
DEBUGFLAGS = ' ';
%DEBUGFLAGS = ' -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

NAME = 'walkgen_sfun';
%% Compile
eval(['mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ', MPC_WALKGEN_OBJECTS, QPOASES_OBJECTS]]);
disp([NAME, '.', eval('mexext'), ' successfully created!']);

%% Set path and copy libraries
path(path, pwd);
%path(path, [pwd,'/',MPC_WALKGEN_LIBRARY_PATH]);
% Copy libraries
%copyfile([MPC_WALKGEN_LIBRARY_PATH,'\mpc-walkgen.dll'],[pwd,'/mpc-walkgen.dll']);
%copyfile([MPC_WALKGEN_LIBRARY_PATH,'\mpc-walkgen.lib'],[pwd,'/mpc-walkgen.lib']);
%% Clear
clear EIGEN_PATH QPOASES_PATH QPOASES_LIB_PATH MPC_WALKGEN_PATH MPC_WALKGEN_LIBRARY_PATH IFLAGS CPPFLAGS MPC_WALKGEN_OBJECTS DEBUGFLAGS NAME