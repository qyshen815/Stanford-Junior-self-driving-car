include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 		-lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		PROCESS_VIEW

SOURCES = 		test_ipc.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	test_ipc pidcontrol_gui

QT_SOURCES = 		proccontrol_gui.cpp proccontrol_gui_moc.cpp
QT_TARGETS = 		pidcontrol_gui

TARGETS = 		test_ipc 

test_ipc:		test_ipc.o

pidcontrol_gui:   	proccontrol_gui.o proccontrol_gui_moc.o

include ../../Makefile.rules
