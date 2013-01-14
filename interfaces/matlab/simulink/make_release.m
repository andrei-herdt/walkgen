%%	Filename:  interfaces/matlab/simulink/make.m
%%	Author:    Andrei Herdt
%%	Version:
%%	Date:      2012
%%

if (ispc == 0)
    CFLAGS  = ['-lblas -lstdc++ -D__cpluplus -D__MATLAB__ -O -DLINUX', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
else
    CFLAGS  = [' -D__WIN32__',' -D__NO_COPYRIGHT__',' -D__SUPPRESSANYOUTPUT__',' -D__STDC__',' -O', ' ' ]; 
end

DEBUGFLAGS = ' ';