include ../../Makefile.conf

ifeq ($(ARCH), Darwin)
  IFLAGS += -I/Library/Frameworks/Gtk.framework/Headers
  IFLAGS += -I/Library/Frameworks/GLib.framework/Headers
  IFLAGS += -I/Library/Frameworks/Cairo.framework/Headers
  LFLAGS += -framework Gtk -framework GLib
else
  IFLAGS += `pkg-config gtk+-2.0 --cflags`
  LFLAGS += `pkg-config gtk+-2.0 --libs`
endif

LFLAGS += 		-lgtk_support $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface

MODULE_NAME = 		LASER_VIEW

SOURCES = 		laser_view.cc riegl_view.cc ibeo_view.cc ldlrs_view.cc
PUBLIC_BINARIES = 	laser_view riegl_view ibeo_view ldlrs_view

TARGETS = 		laser_view riegl_view ibeo_view ldlrs_view

# rules

laser_view:	laser_view.o

riegl_view:	riegl_view.o

ibeo_view:	ibeo_view.o

ldlrs_view:	ldlrs_view.o

include ../../Makefile.rules
