%%	Filename:  interfaces/matlab/simulink/make.m
%%	Author:    Andrei Herdt
%%	Version:
%%	Date:      2012
%%

if (ispc == 0)
    CFLAGS  = [ '-lblas -lstdc++ -D__cpluplus -D__MATLAB__ -O -D__LINUX__', ' ' ];
else
    CFLAGS  = ['-v', ' -D__WIN32__',' -D__NO_COPYRIGHT__',' -D__SUPPRESSANYOUTPUT__',' -D__STDC__', ' ' ]; 
end

% DEBUGFLAGS = ' ';
DEBUGFLAGS = ['-D_DEBUG -g',' '];
%DEBUGFLAGS = ' -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';
