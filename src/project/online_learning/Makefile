
############################################################
# -- Global settings.
############################################################

SHELL			:= /bin/bash
RACE_ROOT		:= ../../..
export ROS_PACKAGE_PATH	:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

# These must exist.
DATA_DIR := cached_descriptors
SEED_MBD := seed.tm.mbd
MULTIBOOSTER := best_tbssl_classifier.mb

all:
	@echo You should probably be more specific.


############################################################
# -- Programs
############################################################

PROJECT_CACHED_DESCRIPTORS	:= $(RACE_ROOT)/src/program/online_learning/project_cached_descriptors
GENERATE_RAW_OD			:= $(RACE_ROOT)/src/program/online_learning/generate_raw_od	
SIMPLE_TEST			:= $(RACE_ROOT)/src/program/online_learning/simple_test
TIMING_EXPERIMENT		:= rosrun online_learning timing_experiment
LABEL_AND_PROJECT		:= $(RACE_ROOT)/src/program/online_learning/label_and_project
EVALUATE_SUPERVISED		:= rosrun online_learning evaluate_supervised


############################################################
# -- Params
############################################################

SEED		:= 0
NUM_PROJECTIONS := 200
NUM_CELLS	:= 1000

TESTING := 	dish_area02-11-17-2009_20-13-49 \
		page_mill_and_hansen01-11-17-2009_19-49-18 \
		page_mill_and_hansen02-11-17-2009_19-56-57 \
		white_plaza02-11-17-2009_17-35-42 \
		driving_test_set-11-17-2009_19-13-35

TRAINING := 	pass2-10-16-2009_15-00-46 \
		intersection-04-07-2010_09-21-53 \
		lasuen02-11-17-2009_17-07-43 \
		CampusLoop3-10-03-2009_15-32-37-FirstMinute \
		lomita_and_santa_teresa01-11-17-2009_18-35-12 \
		intersection-04-07-2010_09-05-08

BACKGROUND :=	null20-11-16-2009_21-12-04 \
		null02-11-16-2009_20-22-34 \
		null06-11-16-2009_20-30-22 \
		null09-11-16-2009_20-33-22 \
		null10-11-16-2009_20-40-17 \
		null15-11-16-2009_21-05-30 \
		null16-11-16-2009_21-07-32 \
		null17-11-16-2009_21-09-21 \
		null18-11-16-2009_21-10-09

UNLABELED :=	logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
		logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
		logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
		logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
		logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
		logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15 \
		logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
		logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
		logs/2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40 \
		logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17 \
		logs/2011-04-07_hoover/overhead9-04-07-2011_15-24-04 \
		logs/2011-04-07_hoover/overhead5-04-07-2011_13-53-33 \
		logs/2011-04-07_hoover/overhead1-04-07-2011_12-55-40 \
		logs/2011-04-07_hoover/overhead7-04-07-2011_14-21-30 \
		logs/2011-04-07_hoover/overhead6-04-07-2011_14-07-56 \
		logs/2011-04-07_hoover/overhead8-04-07-2011_15-10-29 \
		logs/2011-04-07_hoover/overhead2-04-07-2011_13-09-21 \
		logs/2011-04-07_hoover/overhead4-04-07-2011_13-39-58 \
		logs/2011-04-07_hoover/overhead3-04-07-2011_13-23-01

TRAINING_MBDS	:= $(addprefix $(DATA_DIR)/background/, $(addsuffix .tm.mbd, $(BACKGROUND))) $(addprefix $(DATA_DIR)/natural/, $(addsuffix .tm.mbd, $(TRAINING))) 
TESTING_MBDS	:= $(addprefix $(DATA_DIR)/natural/, $(addsuffix .tm.mbd, $(TESTING)))

BACKGROUND_ODS	:= $(addprefix workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/, $(addsuffix .od, $(BACKGROUND)))
TRAINING_ODS	:= $(addprefix workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/, $(addsuffix .od, $(TRAINING)))
TESTING_ODS	:= $(addprefix workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/, $(addsuffix .od, $(TESTING)))
SEED_ODS	:= workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/seed.od

ALL_TRAINING	:= workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/training.od
ALL_TESTING	:= workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/testing.od

RAW_TRAINING	:= workspace/datasets/raw/training.od
RAW_TESTING 	:= workspace/datasets/raw/testing.od

############################################################
# -- Latex
############################################################

latex/clean:
	cd $(@D) && rm icra2012.{aux,bbl,blg,log,pdf,bib} texput.log latex_macros.aux

TEX_TURDS := icra2012.aux icra2012.bbl icra2012.blg icra2012.dvi icra2012.log icra2012.out icra2012.ps latex_macros.aux
latex/icra2012.pdf:	latex/icra2012.tex \
			latex/latex_macros.tex \
			latex/icra2012.bib
	cd $(@D) && pdflatex icra2012.tex
	cd $(@D) && bibtex icra2012
	cd $(@D) && pdflatex icra2012.tex
	cd $(@D) && pdflatex icra2012.tex
	cd $(@D) && rm icra2012.{aux,bbl,blg,log} texput.log latex_macros.aux

latex/icra2012.bib: 	latex/paper_refs-ICRA2012.bib \
			latex/process_bib.py
#		cd $(@D) && python process_bib.py paper_refs-ICRA2012.bib | grep -v 'isbn = ' | grep -v 'doi = ' | grep -v 'issn = ' | grep -v 'url = ' | sed '/annote = {.*},/d' | sed '/annote/,/},/d' | grep -v 'file = ' > icra2012.bib
	cp $< $@

############################################################
# -- Experiments
############################################################

hybrid_stochastic_results: workspace/hybrid_stochastic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results
logistic_stochastic_results: workspace/logistic_stochastic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results
logistic_results: workspace/logistic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results
naive_results: workspace/naive/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results

workspace/logistic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results: $(ALL_TRAINING) $(ALL_TESTING)
	mkdir -p $(@D)
	MAX_NUM_ITERS=100 LOGISTIC= NUM_CELLS=$(NUM_CELLS) $(SIMPLE_TEST) --train='$(ALL_TRAINING)' --test='$(ALL_TESTING)' --output=$@

workspace/logistic_stochastic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results: $(ALL_TRAINING) $(ALL_TESTING)
	mkdir -p $(@D)
	LOGISTIC_STOCHASTIC= NUM_CELLS=$(NUM_CELLS) $(SIMPLE_TEST) --train='$(ALL_TRAINING)' --test='$(ALL_TESTING)' --output=$@

workspace/hybrid_stochastic/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results: $(ALL_TRAINING) $(ALL_TESTING)
	mkdir -p $(@D)
	HYBRID_STOCHASTIC= NUM_CELLS=$(NUM_CELLS) $(SIMPLE_TEST) --train='$(ALL_TRAINING)' --test='$(ALL_TESTING)' --output=$@

workspace/naive/$(NUM_CELLS)cells_$(NUM_PROJECTIONS)projections_seed$(SEED)/results: $(ALL_TRAINING) $(ALL_TESTING)
	mkdir -p $(@D)
	NUM_CELLS=$(NUM_CELLS) $(SIMPLE_TEST) --train='$(ALL_TRAINING)' --test='$(ALL_TESTING)' --output=$@

workspace/naive/$(NUM_CELLS)cells_raw/results: $(RAW_TRAINING) $(RAW_TESTING)
	mkdir -p $(@D)
	NUM_CELLS=$(NUM_CELLS) $(SIMPLE_TEST) --train='$(RAW_TRAINING)' --test='$(RAW_TESTING)' --output=$@

workspace/twiddle_logistic:
	for num_projections in `seq -f'%02g' 1 2 21`; do \
	for num_cells in 100 1000 10000; do \
	NUM_PROJECTIONS=$$num_projections NUM_CELLS=$$num_cells make -e -f Makefile logistic_results; \
	done; \
	done;

workspace/twiddle_naive:
	for num_projections in `seq -f'%02g' 1 2 21`; do \
	for num_cells in 100 1000 10000; do \
	NUM_PROJECTIONS=$$num_projections NUM_CELLS=$$num_cells make -e -f Makefile naive_results; \
	done; \
	done;

############################################################
# -- Supervised learning experiment
############################################################

# Put in this order for stochastic logistic.
SUPERVISED_TRAINING_ODS :=	workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead9-04-07-2011_15-24-04.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead5-04-07-2011_13-53-33.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead1-04-07-2011_12-55-40.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead7-04-07-2011_14-21-30.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead6-04-07-2011_14-07-56.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead8-04-07-2011_15-10-29.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead2-04-07-2011_13-09-21.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead4-04-07-2011_13-39-58.od \
				workspace/datasets/automatically_labeled/logs/2011-04-07_hoover/overhead3-04-07-2011_13-23-01.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null20-11-16-2009_21-12-04.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null06-11-16-2009_20-30-22.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null10-11-16-2009_20-40-17.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null16-11-16-2009_21-07-32.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null18-11-16-2009_21-10-09.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null09-11-16-2009_20-33-22.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null15-11-16-2009_21-05-30.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null17-11-16-2009_21-09-21.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50.od \
				workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null02-11-16-2009_20-22-34.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06.od \
				workspace/datasets/automatically_labeled/logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17.od

SUPERVISED_CV_ODS	:= $(TRAINING_ODS)
SUPERVISED_HOLDOUT_ODS	:= $(TRAINING_ODS)

%-experiment_metadata.txt:
	mkdir -p $(@D)
	echo `hostname` > $@
	echo `date +%F_%T` >> $@
	echo '' >> $@
	hg tip >> $@
	echo '' >> $@
	env >> $@

workspace/supervised/stochastic_logistic: $(SUPERVISED_CV_ODS) $(SUPERVISED_TRAINING_ODS) $(SUPERVISED_HOLDOUT_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt
	$(EVALUATE_SUPERVISED) \
	--method stochastic_logistic \
	$(addprefix --cv , $(SUPERVISED_CV_ODS)) \
	$(addprefix --train , $(SUPERVISED_TRAINING_ODS) ) \
	$(addprefix --holdout , $(SUPERVISED_HOLDOUT_ODS)) \
	$(addprefix --test , $(TESTING_ODS)) \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/

workspace/supervised/stochastic_hybrid: $(SUPERVISED_CV_ODS) $(SUPERVISED_TRAINING_ODS) $(SUPERVISED_HOLDOUT_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt
	$(EVALUATE_SUPERVISED) \
	--method stochastic_hybrid \
	$(addprefix --cv , $(SUPERVISED_CV_ODS)) \
	$(addprefix --train , $(SUPERVISED_TRAINING_ODS) ) \
	$(addprefix --holdout , $(SUPERVISED_HOLDOUT_ODS)) \
	$(addprefix --test , $(TESTING_ODS)) \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/

workspace/supervised/stochastic_logistic-hand_labeled_only: $(TRAINING_ODS) $(BACKGROUND_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt
	BYPASS_CV= INCREMENTAL_EVAL= $(EVALUATE_SUPERVISED) \
	--method stochastic_logistic \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/pass2-10-16-2009_15-00-46.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null09-11-16-2009_20-33-22.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null20-11-16-2009_21-12-04.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null15-11-16-2009_21-05-30.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null06-11-16-2009_20-30-22.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lasuen02-11-17-2009_17-07-43.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/pass2-10-16-2009_15-00-46.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null09-11-16-2009_20-33-22.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null20-11-16-2009_21-12-04.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null15-11-16-2009_21-05-30.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null06-11-16-2009_20-30-22.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lasuen02-11-17-2009_17-07-43.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null10-11-16-2009_20-40-17.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null17-11-16-2009_21-09-21.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/CampusLoop3-10-03-2009_15-32-37-FirstMinute.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null16-11-16-2009_21-07-32.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null02-11-16-2009_20-22-34.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lomita_and_santa_teresa01-11-17-2009_18-35-12.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null18-11-16-2009_21-10-09.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-05-08.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/pass2-10-16-2009_15-00-46.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null09-11-16-2009_20-33-22.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null20-11-16-2009_21-12-04.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null15-11-16-2009_21-05-30.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null06-11-16-2009_20-30-22.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lasuen02-11-17-2009_17-07-43.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null10-11-16-2009_20-40-17.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null17-11-16-2009_21-09-21.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/CampusLoop3-10-03-2009_15-32-37-FirstMinute.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null16-11-16-2009_21-07-32.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null02-11-16-2009_20-22-34.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lomita_and_santa_teresa01-11-17-2009_18-35-12.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/background/null18-11-16-2009_21-10-09.od \
	--holdout workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-05-08.od \
	$(addprefix --test , $(TESTING_ODS)) \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/

workspace/supervised/naive: $(SUPERVISED_CV_ODS) $(SUPERVISED_TRAINING_ODS) $(SUPERVISED_HOLDOUT_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt
	$(EVALUATE_SUPERVISED) \
	--method naive \
	$(addprefix --cv , $(SUPERVISED_CV_ODS)) \
	$(addprefix --train , $(SUPERVISED_TRAINING_ODS) ) \
	$(addprefix --holdout , $(SUPERVISED_HOLDOUT_ODS)) \
	$(addprefix --test , $(TESTING_ODS)) \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/

workspace/supervised/small_scale: $(SUPERVISED_CV_ODS) $(SUPERVISED_TRAINING_ODS) $(SUPERVISED_HOLDOUT_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt 
	mkdir -p $(@D)
	$(EVALUATE_SUPERVISED) \
	--method stochastic_logistic \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-05-08.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-05-08.od \
	--holdout workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--test workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/dish_area02-11-17-2009_20-13-49.od \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/

workspace/supervised/naive-hand_labeled_only: $(TRAINING_ODS) $(BACKGROUND_ODS) $(TESTING_ODS) $$@-experiment_metadata.txt
	$(EVALUATE_SUPERVISED) \
	--method naive \
	$(addprefix --train , $(BACKGROUND_ODS) ) \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--cv workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lomita_and_santa_teresa01-11-17-2009_18-35-12.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-05-08.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lasuen02-11-17-2009_17-07-43.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/CampusLoop3-10-03-2009_15-32-37-FirstMinute.od \
	--train workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/pass2-10-16-2009_15-00-46.od \
	--holdout workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/intersection-04-07-2010_09-21-53.od \
	--holdout workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/natural/lomita_and_santa_teresa01-11-17-2009_18-35-12.od \
	$(addprefix --test , $(TESTING_ODS)) \
	--output $@ > $@-log.txt
	mv $@-log.txt $@-experiment_metadata.txt $@/


############################################################
# -- Timing experiment
############################################################

THRESH := 0

workspace/timing_experiment/naive/done_learning: $(SEED_ODS) $(BACKGROUND_ODS) $(TRAINING_ODS)
	mkdir -p workspace/timing_experiment
	$(TIMING_EXPERIMENT) naive $(@D) $(THRESH) --seed $(SEED_ODS) $(BACKGROUND_ODS) --unlabeled $(SEED_ODS) $(TRAINING_ODS) 
	touch $@

############################################################
# -- Dataset setup
############################################################

#datasets:  	$(ALL_TRAINING) $(ALL_TESTING) $(TRAINING_ODS) $(TESTING_ODS) $(BACKGROUND_ODS) $(SEED_ODS)
datasets:  	$(TRAINING_ODS) $(TESTING_ODS) $(BACKGROUND_ODS) $(SEED_ODS)

workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/training.od:
	mkdir -p $(@D)
	$(PROJECT_CACHED_DESCRIPTORS) $(SEED) $(NUM_PROJECTIONS) $(TRAINING_MBDS) $@

workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/testing.od:
	mkdir -p $(@D)
	$(PROJECT_CACHED_DESCRIPTORS) $(SEED) $(NUM_PROJECTIONS) $(TESTING_MBDS) $@

workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/%.od: $(DATA_DIR)/%.tm.mbd
	mkdir -p $(@D)
	$(PROJECT_CACHED_DESCRIPTORS) $(SEED) $(NUM_PROJECTIONS) $< $@

workspace/datasets/$(NUM_PROJECTIONS)projections_seed$(SEED)/seed.od: seed.tm.mbd
	mkdir -p $(@D)
	$(PROJECT_CACHED_DESCRIPTORS) $(SEED) $(NUM_PROJECTIONS) $< $@


RAW_BACKGROUND_ODS	:= $(addprefix workspace/datasets/raw/background/, $(addsuffix .od, $(BACKGROUND)))
RAW_TRAINING_ODS	:= $(addprefix workspace/datasets/raw/natural/, $(addsuffix .od, $(TRAINING)))
RAW_TESTING_ODS		:= $(addprefix workspace/datasets/raw/natural/, $(addsuffix .od, $(TESTING)))
RAW_SEED_ODS		:= workspace/datasets/raw/seed.od

raw_datasets:	$(RAW_BACKGROUND_ODS) $(RAW_TRAINING_ODS) $(RAW_TESTING_ODS) $(RAW_SEED_ODS)

workspace/datasets/raw/%.od: $(DATA_DIR)/%.tm.mbd
	mkdir -p $(@D)
	$(GENERATE_RAW_OD) $< $@

workspace/datasets/raw/seed.od: seed.tm.mbd
	mkdir -p $(@D)
	$(GENERATE_RAW_OD) $< $@


automatically_labeled_datasets:	$(addprefix workspace/datasets/automatically_labeled/, $(addsuffix .od, $(UNLABELED)))

workspace/datasets/automatically_labeled/%.od: $(DATA_DIR)/%.tm.mbd
	mkdir -p $(@D)
	NUM_THREADS=20 SEED=$(SEED) NUM_PROJECTIONS=$(NUM_PROJECTIONS) $(LABEL_AND_PROJECT) $(MULTIBOOSTER) $< $@