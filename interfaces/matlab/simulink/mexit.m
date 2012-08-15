% ATT: this links statically against the mpc-walkgen.lib and dynamically
% against mpc-walkgen.dll
copyfile('D:\andrei\mpc-walkgen\build-new\Debug\mpc-walkgen.dll','D:\andrei\mpc-walkgen\unittest\use-solver\mpc-walkgen.dll')
copyfile('D:\andrei\mpc-walkgen\build-new\Debug\mpc-walkgen.lib','D:\andrei\mpc-walkgen\unittest\use-solver\mpc-walkgen.lib')
mex -g -D_DEBUG -v -ID:\andrei\mpc-walkgen\include -ID:\andrei\mpc-walkgen\deps\eigen\ -LD:\andrei\mpc-walkgen\build-new\Debug\ walkgen_cpp.cpp mpc-walkgen.lib