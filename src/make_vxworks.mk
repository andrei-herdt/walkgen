##	Filename:  SRC/make_vxworks
##	Author:    Andrei Herdt
##


##
##	definitions for compiling with gcc under vxworks
##

CPP = /home/f_legs/devel/new_wrapper/linux/bin/vxworks6.9-x86-gcc4.x-g++
AR  = /home/f_legs/devel/new_wrapper/linux/bin/vxworks6.9-x86-gcc4.x-ar
RM  = rm

OBJEXT = o
LIBEXT = a
EXE = 
DEF_TARGET = -o $@

CPPFLAGS = -g -Wall -Wfloat-equal -O3 -finline-functions -D__VXWORKS__ -DNDEBUG
#CPPFLAGS = -g -D__DEBUG__ -D__VXWORKS__

MPC_WALKGEN_LIB         =  -L${SRCDIR} -lmpc-walkgen


##
##	end of file
##
