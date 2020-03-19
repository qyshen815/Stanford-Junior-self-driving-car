TARBALL = build/DevIL-1.7.8.tar.gz
TARBALL_URL = http://downloads.sourceforge.net/openil/DevIL-1.7.8.tar.gz
SOURCE_DIR = build/devil-1.7.8
#TARBALL_PATCH = patchfile
all: $(SOURCE_DIR)/unpacked
	if [ ! -f $(SOURCE_DIR)/lib/.libs/libIL.so ]; then echo "Running configure & make"; cd $(SOURCE_DIR) && ./configure --enable-jpeg && make; fi

include $(shell rospack find mk)/download_unpack_build.mk

clean: 
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build

