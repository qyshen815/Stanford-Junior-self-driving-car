TARFILE = octomap-0.42.tar.gz
BUILD_DIR = octomap-0.42

all: $(BUILD_DIR)
	cd $(BUILD_DIR)/build && cmake ../src && make

$(BUILD_DIR): $(TARFILE)
	tar xvzf $(TARFILE)

$(TARFILE):
	wget http://downloads.sourceforge.net/project/octomap/octomap-0.42.tar.gz?use_mirror=hivelocity

clean:
	-rm -rf $(BUILD_DIR)

wipe: clean
	-rm -rf $(TARFILE)
