include ../../Makefile.conf

CXXFLAGS += 
IFLAGS += -I/usr/include/qwt
LFLAGS += -lqwt $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = HEALTHMON_VIEW

SOURCES = 

HT_GUI_FILES = ht-widget.h ht-widget.cpp ht-widget-moc.cpp ht-view-moc.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = healthmon_view

QT_SOURCES = ht-main.cpp ht-view.cpp
QT_TARGETS = healthmon_view
TARGETS = 

EXTRA_CLEAN += $(HT_GUI_FILES)

# rules

depend:			$(HT_GUI_FILES)

healthmon_view:		$(HT_GUI_FILES) ht-main.o ht-widget.o \
			ht-widget-moc.o ht-view.o ht-view-moc.o

include ../../Makefile.rules
