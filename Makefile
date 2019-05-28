NAME_SRV = gigecam-srv

export PYLON_ROOT=/opt/pylon-3.0.0
export GENICAM_ROOT=${PYLON_ROOT}/genicam

PYLON_LIBS = -L${PYLON_ROOT}/lib64 -L${PYLON_ROOT}/lib  -lpylonbase
GENICAM_LIBS = -L${GENICAM_ROOT}/bin/Linux64_x64 -L${GENICAM_ROOT}/bin/Linux64_x64/GenApi/Generic -lXerces-C_gcc40_v2_7 -lGenApi_gcc40_v2_3 -lGCBase_gcc40_v2_3 -lMathParser_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3


CXXFLAGS = -I${GENICAM_ROOT}/library/CPP/include -I${PYLON_ROOT}/include -O2 -pipe -fomit-frame-pointer -DUSE_GIGE -DLIBFIT -DCONFIG_64 -DGIGECAM_RNM
LDFLAGS =   ${PYLON_LIBS} ${GENICAM_LIBS} -Wl,-E -lbufferrt -lrt -lfit -lgsl -lgslcblas -lrnm -lrnmshare

include ../makefiles/Make-8.1.2.c.in
