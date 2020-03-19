include ../../Makefile.conf

ifeq ($(ARCH), Darwin)
CFLAGS += -Wno-error	# required because of strange function declaration
endif

IFLAGS += 
LFLAGS += 	-lsignal_handler $(INTERFACE_LIBS) $(GLOBAL_LIBS)


MODULE_NAME = 		APPLANIX

SOURCES = 		applanix.cc applanixcore.cc applanix_logger.cc \
			applanix_integrate.cc fake_applanix.cc

ifeq ($(ARCH), Darwin)
#SOURCES += macAddOns.cc
endif

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	applanix applanix_logger applanix_integrate fake_applanix

TARGETS = 		applanix applanix_logger applanix_integrate fake_applanix

# rules

ifeq ($(ARCH), Darwin)
applanix:	applanix.o applanixcore.o #macAddOns.o
applanix_logger:	applanix_logger.o applanixcore.o #macAddOns.o
else
applanix:		applanix.o applanixcore.o
applanix_logger:	applanix_logger.o applanixcore.o
endif

fake_applanix: fake_applanix.o
applanix_integrate:	applanix_integrate.o

include ../../Makefile.rules
