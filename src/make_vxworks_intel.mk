##	Filename:  SRC/make_vxworks
##	Author:    Andrei Herdt


##
##	definitions for compiling with icc under vxworks
##

CPP = /opt/vxworks/wrapper/linux/bin/vxworks6.9smp-x86-icc12.x-icpc
AR  = /opt/vxworks/wrapper/linux/bin/vxworks6.9smp-x86-icc12.x-xiar
RM  = rm

OBJEXT = o
LIBEXT = a
EXE = 
DEF_TARGET = -o $@

CPPFLAGS = -g -Wall -xAVX -O3 -finline-functions -D__VXWORKS__ -DNDEBUG
#CPPFLAGS = -g -D__DEBUG__ -D__VXWORKS__

MPC_WALKGEN_LIB         =  -L${SRCDIR} -lmpc-walkgen


##
##	end of file
##
