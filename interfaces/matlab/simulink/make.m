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
MPC_WALKGEN_LIBRARY_PATH = [MPC_WALKGEN_PATH,'build-new/Debug/'];
EIGEN_PATH = [MPC_WALKGEN_PATH,'deps/eigen/'];


IFLAGS  = ['-I. -I',MPC_WALKGEN_PATH,'include',' -I',EIGEN_PATH,' -L',MPC_WALKGEN_LIBRARY_PATH,' ' ];

if ( ispc == 0 )
	%CPPFLAGS  = [ IFLAGS, '-D__cpluplus -D__MATLAB__ -O -DLINUX', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
else
	CPPFLAGS  = [IFLAGS, '-v', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
end

MPC_WALKGEN_OBJECTS = [MPC_WALKGEN_LIBRARY_PATH,'mpc-walkgen.lib ', ' '];

DEBUGFLAGS = ['-D_DEBUG -g',' '];
%DEBUGFLAGS = ' -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

NAME = 'walkgen_sfun';
%% Compile
eval(['mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',MPC_WALKGEN_OBJECTS]]);
disp([NAME, '.', eval('mexext'), ' successfully created!']);

%% Set path and copy libraries
path(path, pwd);
path(path, [pwd,'/',MPC_WALKGEN_LIBRARY_PATH]);
% Necessary right now
copyfile([MPC_WALKGEN_LIBRARY_PATH,'\mpc-walkgen.dll'],[pwd,'/mpc-walkgen.dll']);
copyfile([MPC_WALKGEN_LIBRARY_PATH,'\mpc-walkgen.lib'],[pwd,'/mpc-walkgen.lib']);
%% Clear
clear EIGEN_PATH MPC_WALKGEN_PATH MPC_WALKGEN_LIBRARY_PATH IFLAGS CPPFLAGS MPC_WALKGEN_OBJECTS DEBUGFLAGS NAME
%%
%%	end of file
%%
