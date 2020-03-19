TARBALL = build/qpOASES-2.0.tar.gz
#TARBALL_URL = http://www.kuleuven.be/optec/software/qpOASES/download/qpOASES-2.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/qpOASES-2.0.tar.gz
SOURCE_DIR = build/qpOASES-2.0
TARBALL_PATCH = patchfile
all: $(SOURCE_DIR)/unpacked
	cd build/qpOASES-2.0/SRC && make -f Makefile
	-rm -rf include
	-mkdir include
	cd include && ln -s ../build/qpOASES-2.0/INCLUDE qpOASES

include $(shell rospack find mk)/download_unpack_build.mk

clean: 
	-rm -rf $(SOURCE_DIR) include

wipe: clean
	-rm -rf build
