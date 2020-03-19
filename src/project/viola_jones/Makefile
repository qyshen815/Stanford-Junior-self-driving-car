
classifier: samples.vec background.txt
	rosrun opencv2 opencv_haartraining -data $@ -vec samples.vec -bg background.txt -npos `cat faces.txt | wc -l` -nneg `cat background.txt | wc -l` -mem 2000 -w 19 -h 19

samples.vec: faces.txt
	rosrun opencv2 opencv_createsamples -info faces.txt -vec samples.vec -w 19 -h 19 -num `cat faces.txt | wc -l`

background.txt:
	-rm $@
	for file in `find data/train/non-face/ -name '*.pgm'`; do \
	echo $$file >> $@; done

faces.txt: data
	-rm $@
	for file in `find data/train/face/ -name '*.pgm' | awk '{if(NR < 101) print $$0}'`; do \
	echo $$file 1 0 0 19 19 >> $@; done
	for file in `find data/train/negface/ -name '*.pgm' | awk '{if(NR < 101) print $$0}'`; do \
	echo $$file 1 0 0 19 19 >> $@; done

data: faces.tar.gz
	mkdir $@
	mv faces.tar.gz $@/
	cd $@ && tar xvzf faces.tar.gz && mv faces.tar.gz ../
	cd $@ && tar xvzf face.train.tar.gz
	cd $@/train && cp -r face negface
	chmod a+w -R $@
	for img in `ls $@/train/negface/*.pgm`; do convert $$img -negate $$img; done

faces.tar.gz:
	wget http://cbcl.mit.edu/projects/cbcl/software-datasets/faces.tar.gz

clean:
	-rm -rf data faces.txt background.txt samples.vec classifier* 

wipe: clean
	-rm -rf faces.tar.gz
