include ../../Makefile.conf

CXXFLAGS +=
#IFLAGS += $(SUPERLU_IFLAGS)
#LFLAGS += $(SUPERLU_LFLAGS)

#LFLAGS += -lrtc_base

MODULE_NAME = RTC Math Libary

SOURCES = rtcPose2D.cpp rtcPose3D.cpp

PUBLIC_INCLUDES = rtcMath.h rtcArray1.h rtcEulerAngles.h rtcSMat2.h rtcTransform.h \
                  rtcArray2.h rtcSMat3.h rtcVec2.h \
                  rtcArray3.h rtcMat.h rtcQuaternion.h rtcSMat4.h rtcSMat5.h \
                  rtcSMat6.h rtcVec3.h rtcVec5.h \
                  rtcArray.h rtcRotation2D.h rtcSMat.h rtcVec4.h rtcVec6.h \
                  rtcRotation.h rtcTransform2D.h rtcVec.h \
                  rtcPose2D.h rtcPose3D.h rtcPoint2D.h rtcPoint3D.h \
                  rtcVarMat.h rtcVarSMat.h rtcVarVec.h
                  
                  # LinearSystem.h EigenSystem.h  VarLinearSystem.h \
                  #SparseMat.h SparseSMat.h SparseLinearSystem.h \
                  #SuperLUWrapper.h SingularValueDecomposition.h \
                  #Sort.h PCA.h 

PUBLIC_LIBRARIES = librtc_math.a#$(EXT)
PUBLIC_BINARIES = 

TARGETS = librtc_math.a#$(EXT)

# rules
librtc_math.a: rtcPose2D.o #rtcPose3D.o
#librtc_math.$(EXT): rtcPose2D.o rtcPose3D.o
              
include ../../Makefile.rules
