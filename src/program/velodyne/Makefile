include ../../Makefile.conf

CXXFLAGS += 
IFLAGS += 
LFLAGS += -lvelo_support $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lasync_writer -lrt \
	  -lsignal_handler -lipc_interface
LINK = g++

MODULE_NAME = VELODYNE

SOURCES = 	velodyne_main.cpp velodyne_logger.cpp velodyne_stat.cc \
		velodyne_dump.cpp velocomm.cpp vlf_index.cpp vlf_cut.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = velodyne velodyne_logger vlf_index vlf_cut

TARGETS = 	velodyne velodyne_logger velodyne_stat velodyne_dump vlf_index vlf_cut

# rules

velodyne:		velodyne_main.o velocomm.o

velodyne_logger:	velodyne_logger.o 

velodyne_stat:		velodyne_stat.o

velodyne_dump:		velodyne_dump.o

vlf_index:		vlf_index.o

vlf_cut:		vlf_cut.o
include ../../Makefile.rules
