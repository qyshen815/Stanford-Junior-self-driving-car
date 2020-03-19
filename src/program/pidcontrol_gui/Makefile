UIC_BINS = /usr/bin/uic /usr/bin/uic-qt4
MOC_BINS = /usr/bin/moc /usr/bin/moc-qt4

QT_UIC = $(shell for f in $(UIC_BINS) ; do if [ -x $$f ]; then echo "$$f"; break; fi; done;)
QT_MOC = $(shell for f in $(MOC_BINS) ; do if [ -x $$f ]; then echo "$$f"; break; fi; done;)

QT_INCLUDES = -I/usr/include/QtCore -I/usr/include/QtGui
QT_LIBS = -lQtGui -lrote

include ../../Makefile.conf

LFLAGS += -lmulticentral $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = PIDCONTROL_GUI

SOURCES =

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = pidcontrol_gui

QT_SOURCES = qflow.cpp qbutton.cpp pidcontrol_gui.cpp
QT_OBJECTS += qflow.o qbutton.o qbutton-moc.o pidcontrol_gui.o pidcontrol_gui-moc.o
QT_GUI_FILES = qbutton-moc.cpp pidcontrol_gui-moc.cpp
EXTRA_CLEAN += $(QT_GUI_FILES)

QT_TARGETS = pidcontrol_gui
GRAPHICS_TARGETS = pidcontrol_gui
TARGETS =  

# rules

pidcontrol_gui:   	qflow.o qbutton.o qbutton-moc.o pidcontrol_gui-moc.o pidcontrol_gui.o


include ../../Makefile.rules
