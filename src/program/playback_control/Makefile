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

LFLAGS += -lgtk_support -lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = 		PLAYBACK_CONTROL

PUBLIC_BINARIES = 	dgc_playback_cmd dgc_playback_play dgc_playback_stop \
		  	dgc_playback_reset dgc_playback_control
SOURCES = 		dgc_playback_cmd.cc dgc_playback_control.cc

TARGETS = 		dgc_playback_cmd dgc_playback_play dgc_playback_stop \
	  		dgc_playback_reset dgc_playback_control

# rules

dgc_playback_cmd:	dgc_playback_cmd.o

dgc_playback_control:	dgc_playback_control.o

dgc_playback_play:
	ln -s dgc_playback_cmd dgc_playback_play

dgc_playback_stop:
	ln -s dgc_playback_cmd dgc_playback_stop

dgc_playback_reset:
	ln -s dgc_playback_cmd dgc_playback_reset

include ../../Makefile.rules
