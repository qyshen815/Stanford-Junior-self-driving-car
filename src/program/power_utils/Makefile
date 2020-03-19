include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = 		POWER_UTILS

SOURCES = 		power_set.cc dgc_powerup.cc dgc_powerdown.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	power_set dgc_powerup dgc_powerdown power_gui

QT_SOURCES = 		power_gui.cpp power_gui_moc.cpp
QT_TARGETS = 		power_gui
TARGETS = 		power_set dgc_powerup dgc_powerdown 

# rules

power_set:		power_set.o

dgc_powerup:		dgc_powerup.o

dgc_powerdown:		dgc_powerdown.o

power_gui:   		power_gui.o power_gui_moc.o

include ../../Makefile.rules
