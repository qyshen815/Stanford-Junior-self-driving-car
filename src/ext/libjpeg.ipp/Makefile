include ../../Makefile.conf
##############################################################################
#  Makefile
#    used to build IJG sample
#
ifneq ($(ARCH), Darwin)
ARCH = linux32
# LINKAGE=static
endif

include $(ARCH).ini

#ifndef IPPROOT
#  IPPROOT=/opt/intel/ipp/5.0/ia32
#endif

CFLAGS = -O2 -fpic -Dlinux -Dlinux64
IFLAGS = -I$(IPPROOT)/include
LD_LIBRARY_PATH := "$(IPPROOT)/sharedlib:$(LD_LIBRARY_PATH)"

MODULE_NAME = LIBJPEG.IPP

SOURCES = \
  jcapimin.c  jcmarker.c  jdapimin.c  jdinput.c   jdtrans.c   jidctred.c \
  jcapistd.c  jcmaster.c  jdapistd.c  jdmainct.c  jerror.c    jmemmgr.c  \
  jccoefct.c  jcomapi.c   jdatadst.c  jdmarker.c  jfdctflt.c  jmemnobs.c \
  jccolor.c   jcparam.c   jdatasrc.c  jdmaster.c  jfdctfst.c  jquant1.c  \
  jcdctmgr.c  jcphuff.c   jdcoefct.c  jdmerge.c   jfdctint.c  jquant2.c  \
  jchuff.c    jcprepct.c  jdcolor.c   jdphuff.c   jidctflt.c  jutils.c   \
  jcinit.c    jcsample.c  jddctmgr.c  jdpostct.c  jidctfst.c  jcmainct.c \
  jctrans.c   jdhuff.c    jdsample.c  jidctint.c

OBJECTS = \
  jcapimin.o  jcmarker.o  jdapimin.o  jdinput.o   jdtrans.o   jidctred.o \
  jcapistd.o  jcmaster.o  jdapistd.o  jdmainct.o  jerror.o    jmemmgr.o  \
  jccoefct.o  jcomapi.o   jdatadst.o  jdmarker.o  jfdctflt.o  jmemnobs.o \
  jccolor.o   jcparam.o   jdatasrc.o  jdmaster.o  jfdctfst.o  jquant1.o  \
  jcdctmgr.o  jcphuff.o   jdcoefct.o  jdmerge.o   jfdctint.o  jquant2.o  \
  jchuff.o    jcprepct.o  jdcolor.o   jdphuff.o   jidctflt.o  jutils.o   \
  jcinit.o    jcsample.o  jddctmgr.o  jdpostct.o  jidctfst.o  jcmainct.o \
  jctrans.o   jdhuff.o    jdsample.o  jidctint.o
 
PUBLIC_INCLUDES  = jpegipp.h      jinclude.ipp.h  jerror.ipp.h \
    jpeglib.ipp.h  jconfig.ipp.h  jmorecfg.ipp.h  jpegint.ipp.h 

PUBLIC_LIBRARIES = libjpeg.ipp.a 
PUBLIC_BINARIES  = 

TARGETS = libjpeg.ipp.a libjpeg.ipp.so

ifeq ($(LINKAGE),static)
LFLAGS := -L $(IPPROOT)/lib
ifeq ($(ARCH),linuxixp_le)
LFLAGS := -L $(IPPROOT)/lib/le
endif
ifeq ($(ARCH),linuxixp_be)
LFLAGS := -L $(IPPROOT)/lib/be
endif
else
ifeq ($(ARCH), Darwin)
	LFLAGS := -dynamiclib -L$(IPPROOT)/sharedlib
else
LFLAGS := -L $(IPPROOT)/sharedlib -L $(IPPROOT)/sharedlib/$(ARCH)
endif
endif

ifeq ($(LINKAGE),static)
ifeq ($(ARCH),linux32)
IPPLIBS := -lippjemerged$(LIB_ARCH) -lippjmerged$(LIB_ARCH) -lippiemerged$(LIB_ARCH) -lippimerged$(LIB_ARCH) -lippsemerged$(LIB_ARCH) -lippsmerged$(LIB_ARCH) -lippcore$(LIB_ARCH)
endif
ifeq ($(ARCH),linuxem64t)
IPPLIBS := -lippjemerged$(LIB_ARCH) -lippjmerged$(LIB_ARCH) -lippiemerged$(LIB_ARCH) -lippimerged$(LIB_ARCH) -lippsemerged$(LIB_ARCH) -lippsmerged$(LIB_ARCH) -lippcore$(LIB_ARCH)
endif
ifeq ($(ARCH),linux64)
IPPLIBS := -lippji7 -lippii7 -lippsi7 -lippcore$(LIB_ARCH)
endif
ifeq ($(ARCH),linuxixp_le)
IPPLIBS := -lippjs2 -lippis2 -lippss2 -lippcore$(LIB_ARCH)
endif
ifeq ($(ARCH),linuxixp_be)
IPPLIBS := -lippjs2 -lippis2 -lippss2 -lippcore$(LIB_ARCH)
endif
else
IPPLIBS := -lippj$(LIB_ARCH) -lippi$(LIB_ARCH) -lipps$(LIB_ARCH) -lippcore$(LIB_ARCH)
endif

LFLAGS += $(IPPLIBS)
#rules

all:

libjpeg.ipp.a: $(OBJECTS)

libjpeg.ipp.so: $(OBJECTS)
  
## build libjpeg dynamic library
#$(BinDir)/$(LIBPREFIX)$(LibName).$(SOPOSTFIX) : $(LibObj)
#ifeq ($(LINKAGE),static)
#	$(CC) -shared -o $@ $^ $(LDFLAGS) $(IPPLIBS)
#else
#	$(CC) -shared -o $@ $^ $(LDFLAGS) $(IPPLIBS) -ldl
# endif


include ../../Makefile.rules
