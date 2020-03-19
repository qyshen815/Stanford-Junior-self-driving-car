

ZIP = maxflow-v3.01.zip
ZIP_URL = http://vision.csd.uwo.ca/code/maxflow-v3.01.zip

all: installed

build/unpacked:
	mkdir -p $(@D)
	cd build && wget $(ZIP_URL)
	cd build && unzip $(ZIP)
	touch $@

installed: build/unpacked
	cd build && g++ -O3 -DNDEBUG -I. -fPIC -c graph.cpp
	cd build && g++ -O3 -DNDEBUG -I. -fPIC -c maxflow.cpp
	cd build && g++ -O3 -DNDEBUG -shared -o libmaxflow.so graph.o maxflow.o
	mkdir -p lib
	mv build/libmaxflow.so lib/
	mkdir -p include/maxflow/
	mv build/*.h include/maxflow/
	touch $@

clean: 
	-rm -rf installed bin build include lib
