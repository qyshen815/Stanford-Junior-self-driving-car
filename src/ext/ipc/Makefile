include ../../Makefile.conf

CFLAGS = -Wall -Wstrict-prototypes -Wnested-externs -Wmissing-prototypes \
	 -Wmissing-declarations -O3

ifeq ($(ARCH), Darwin)
CFLAGS += -D__DARWIN__
else
CFLAGS += -DREDHAT_52 -DREDHAT_6 -DREDHAT_71
endif

ifndef NO_TCPD
CFLAGS += -DACCESS_CONTROL
LFLAGS = -lwrap

ifneq ($(ARCH), Darwin)
LFLAGS += -lnsl
endif

endif

# Work-around for FC{5,6} malloc bug: use google malloc library if present
# if you have the google malloc library installed on Ubuntu you might have
# to add a link from /usr/lib/libtcmalloc.so.0 to libtcmalloc.so
# Also, you need to link to pthreads...
LFLAGS += $(shell [ -f /usr/lib/libtcmalloc.so.0 ] && echo -ltcmalloc -lpthread)

MODULE_NAME = IPC

SOURCES = ipc.c queryResponse.c marshall.c timer.c list.c behaviors.c \
	tcModError.c datamsg.c formatters.c hash.c idtable.c key.c \
	primFmttrs.c reg.c sendMsg.c tcaMem.c tcaRef.c comModule.c com.c \
	globalM.c globalMUtil.c strList.c modLogging.c modVar.c resMod.c \
	parseFmttrs.c lex.c printData.c	comServer.c dispatch.c msgTap.c \
	recvMsg.c res.c tcerror.c logging.c globalS.c centralIO.c \
	globalVar.c central.c multiThread.c

PUBLIC_INCLUDES = ipc.h
PUBLIC_LIBRARIES = libipc.a 
PUBLIC_BINARIES = central

TARGETS = libipc.a central

# rules

all:

libipc.a: ipc.o queryResponse.o marshall.o timer.o list.o behaviors.o \
	  tcModError.o datamsg.o formatters.o hash.o idtable.o key.o \
	  primFmttrs.o reg.o sendMsg.o tcaMem.o tcaRef.o comModule.o com.o \
	  globalM.o globalMUtil.o strList.o modLogging.o modVar.o resMod.o \
	  parseFmttrs.o lex.o printData.o 

central:  comServer.o dispatch.o msgTap.o recvMsg.o res.o tcerror.o logging.o \
	  globalS.o centralIO.o globalVar.o central.o libipc.a

include ../../Makefile.rules


