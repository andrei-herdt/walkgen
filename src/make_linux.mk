##	Filename:  SRC/make_linux
##	Author:    Andrei herdt
##


##
##	definitions for compiling under linux
##

CPP = g++
AR  = ar
RM  = rm

OBJEXT = o
LIBEXT = a
EXE =
DEF_TARGET = -o $@

CPPFLAGS = -g -O3 -finline-functions -D__LINUX__ -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__ -march=native -mtune=native

# -ftree-vectorize -msse2 -ftree-vectorizer-verbose=5
#CPPFLAGS = -g -D__DEBUG__ -D__LINUX__

MPC_WALKGEN_LIB         =  -L${SRCDIR} -lmpc-walkgen
THREAD_LIB         =  -lpthread -lgfortran


##
##	end of file
##
