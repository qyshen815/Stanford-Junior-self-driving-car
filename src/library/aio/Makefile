include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lrt -lglobal -lipc
LINK = g++

MODULE_NAME = 		AIO

SOURCES = 		async_writer.cc async_writer_test.cc

PUBLIC_INCLUDES = 	async_writer.h
PUBLIC_LIBRARIES = 	libasync_writer.a
PUBLIC_BINARIES = 

TARGETS = 		libasync_writer.a async_writer_test
TEST_TARGETS =		async_writer_unittest

libasync_writer.a:	async_writer.o

async_writer_unittest:	async_writer_unittest.o libasync_writer.a

async_writer_test:	async_writer_test.o libasync_writer.a

include ../../Makefile.rules
