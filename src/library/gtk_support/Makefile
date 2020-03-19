include ../../Makefile.conf

# delete any fields you don't need

CFLAGS +=
ifeq ($(ARCH), Darwin)
  IFLAGS += -I/Library/Frameworks/Gtk.framework/Headers
  IFLAGS += -I/Library/Frameworks/GLib.framework/Headers
  IFLAGS += -I/Library/Frameworks/Cairo.framework/Headers
  LFLAGS += -framework Gtk
else
  IFLAGS += `pkg-config gtk+-2.0 --cflags`
  LFLAGS += `pkg-config gtk+-2.0 --libs`
endif

LINK =

MODULE_NAME = 		GTK_SUPPORT

SOURCES = 		gtk_support.cpp

PUBLIC_INCLUDES = 	gtk_support.h
PUBLIC_LIBRARIES = 	libgtk_support.a

GTK2_TARGETS = 		libgtk_support.a

# rules

libgtk_support.a:	gtk_support.o

include ../../Makefile.rules
