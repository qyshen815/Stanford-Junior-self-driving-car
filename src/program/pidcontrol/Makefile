include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += $(INTERFACE_LIBS) -lipc_interface $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = PIDCONTROL

SOURCES = pidcontrol.cc pidcontrol_ipc.cc pidcontrol_add.cc \
	  pidcontrol_remove.cc pidcontrol_setmodule.cc pidcontrol_setgroup.cc \
	  crasher.cpp annoyer.cpp test_module.cc view_output.cc \
	  pidcontrol_test.cc test_disconnect.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = pidcontrol pidcontrol_setmodule pidcontrol_setgroup \
	  pidcontrol_add pidcontrol_remove

TARGETS = pidcontrol test_module view_output pidcontrol_setmodule \
	  pidcontrol_setgroup pidcontrol_add pidcontrol_remove \
	  pidcontrol_test test_disconnect crasher annoyer

# rules

pidcontrol:		pidcontrol.o pidcontrol_ipc.o

test_module:		test_module.o

view_output:		view_output.o

pidcontrol_setmodule:	pidcontrol_setmodule.o

pidcontrol_setgroup:	pidcontrol_setgroup.o

pidcontrol_add:		pidcontrol_add.o

pidcontrol_remove:	pidcontrol_remove.o

pidcontrol_test:	pidcontrol_test.o

test_disconnect:	test_disconnect.o

crasher:		crasher.o

annoyer:		annoyer.o

include ../../Makefile.rules
