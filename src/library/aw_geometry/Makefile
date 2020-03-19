include ../../Makefile.conf

CFLAGS += -frounding-math
CXXFLAGS += -frounding-math

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Geometry"

SOURCES = aw_geometry_2d.cpp aw_geometry_3d.cpp

PUBLIC_INCLUDES = aw_geometry.h aw_geometry_2d.h aw_geometry_3d.h \
				  aw_octree.h aw_octree_template.h \
				  aw_CGAL.h
 
PUBLIC_LIBRARIES = libaw_geometry.a
PUBLIC_BINARIES = 

TARGETS = libaw_geometry.a

# rules
libaw_geometry.a: aw_geometry_3d.o aw_geometry_2d.o



include ../../Makefile.rules
