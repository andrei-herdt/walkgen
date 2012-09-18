##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  SRC/make_vxworks
##	Author:    Hans Joachim Ferreau
##	Version:   2.0
##	Date:      2009
##


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
