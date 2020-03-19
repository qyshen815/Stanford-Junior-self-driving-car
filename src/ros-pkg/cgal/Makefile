TARBALL = build/CGAL-3.6.tar.gz
TARBALL_URL = http://cs.stanford.edu/people/teichman/cgal/CGAL-3.6.tar.gz
SOURCE_DIR = build/CGAL-3.6
#TARBALL_PATCH = patchfile
all: $(SOURCE_DIR)/unpacked
	if [ ! -f $(SOURCE_DIR)/lib/libCGAL.so ]; then echo "Running cmake"; cd $(SOURCE_DIR) && cmake . && make; fi

include $(shell rospack find mk)/download_unpack_build.mk

clean: 
	-rm -rf $(SOURCE_DIR) include

wipe: clean
	-rm -rf build CGAL-3.6.tar.gz

