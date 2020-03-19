include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lprotobuf
LINK = g++

MODULE_NAME = 		GOOGLE_PROTOBUF

SOURCES = 		add_person.cc list_people.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = 		add_person list_people

EXTRA_CLEAN =	*.pb.cc *.pb.h

add_person:	add_person.o addressbook.pb.o 

list_people:	list_people.o addressbook.pb.o

include ../../Makefile.rules


