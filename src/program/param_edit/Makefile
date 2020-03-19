include ../../Makefile.conf

#ifeq ($(ARCH), Darwin)
#  IFLAGS += -I/Library/Frameworks/Gtk.framework/Headers
#  IFLAGS += -I/Library/Frameworks/GLib.framework/Headers
#  IFLAGS += -I/Library/Frameworks/Cairo.framework/Headers
#  LFLAGS += -framework Gtk -framework GLib
#else
  IFLAGS += `pkg-config gtk+-2.0 --cflags`
  LFLAGS += `pkg-config gtk+-2.0 --libs`
#endif

LFLAGS +=  -lipc_interface -lgtk_support $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = 		PARAM_EDIT

SOURCES = 		param_set.cc param_edit.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	param_set param_edit

TARGETS = 		param_edit param_set 

# rules

param_set:	param_set.o

param_edit:	param_edit.o

include ../../Makefile.rules
