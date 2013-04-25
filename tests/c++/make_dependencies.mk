EIGEN_DIR = ../../../eigen-3.1.1/ 
SRC_DIR = ../../src/
QPOASES_LIB = ../../../qpoases/bin/libqpOASES.a
BLAS_LIB 	= ../../../qpoases/src/BLASReplacement.o
LAPACK_LIB	= ../../../qpoases/src/LAPACKReplacement.o
I_DIR = ../../include/
QLD_LIB = ../../../qld_dlr/libqld.a

include ../../src/make_linux.mk
#include ../../src/make_vxworks.mk
#include ../../src/make_vxworks_intel.mk
