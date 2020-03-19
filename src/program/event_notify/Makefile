include ../../Makefile.conf

	
MODULE_NAME = EVENT_NOTIFY

ifeq (1,$(HAVE_QT))
  CFLAGS +=
  IFLAGS += $(QT_INCLUDES)
  LFLAGS += $(QT_LIBS) $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface

  SOURCES = event_notify.cpp 
  QT_SOURCES =

  PUBLIC_INCLUDES = 
  PUBLIC_LIBRARIES = 
  PUBLIC_BINARIES = event_notify

  QT_TARGETS =
  TARGETS = event_notify
else
  TARGETS = no_qt
endif
# rules

no_qt: ;@echo " --> Skip event_notify because no qt"

event_notify:  event_notify.o

include ../../Makefile.rules
