include ../../Makefile.conf

CFLAGS +=
IFLAGS += -I/usr/include/qwt
LFLAGS += -lqwt $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = CONTROLLER_VIEW

SOURCES = 

CT_GUI_FILES = ct-widget.h ct-widget.cpp ct-widget-moc.cpp ct-view-moc.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = controller_view

QT_SOURCES = ct-main.cpp ct-view.cpp
QT_TARGETS = controller_view

TARGETS = 

EXTRA_CLEAN += $(CT_GUI_FILES)

# rules

depend:			$(CT_GUI_FILES)

controller_view:	$(CT_GUI_FILES) ct-main.o ct-widget.o \
			ct-widget-moc.o ct-view.o ct-view-moc.o

include ../../Makefile.rules
