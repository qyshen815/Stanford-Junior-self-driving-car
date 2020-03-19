# Global settings.
SHELL := /bin/bash
RACE_ROOT := ../../..
export DESCRIPTORS:=no_rp
export NUM_THREADS:=40
export ROS_PACKAGE_PATH:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

.SECONDEXPANSION: # Allows use of automatic vars in prerequisites.
.SECONDARY: # Prevents deletion of intermediate targets in a chain of implicit rules.

BACKGROUND_FULL :=	background/null20-11-16-2009_21-12-04 \
			background/null02-11-16-2009_20-22-34 \
			background/null06-11-16-2009_20-30-22 \
			background/null09-11-16-2009_20-33-22 \
			background/null10-11-16-2009_20-40-17 \
			background/null15-11-16-2009_21-05-30 \
			background/null16-11-16-2009_21-07-32 \
			background/null17-11-16-2009_21-09-21 \
			background/null18-11-16-2009_21-10-09

TESTING_DATA := 	natural/dish_area02-11-17-2009_20-13-49 \
			natural/page_mill_and_hansen01-11-17-2009_19-49-18 \
			natural/page_mill_and_hansen02-11-17-2009_19-56-57 \
			natural/white_plaza02-11-17-2009_17-35-42 \
			natural/driving_test_set-11-17-2009_19-13-35

LABELED_TRAINING_DATA := 	natural/pass2-10-16-2009_15-00-46 \
				natural/intersection-04-07-2010_09-21-53 \
				natural/lasuen02-11-17-2009_17-07-43 \
				natural/CampusLoop3-10-03-2009_15-32-37-FirstMinute \
				natural/lomita_and_santa_teresa01-11-17-2009_18-35-12 \
				natural/intersection-04-07-2010_09-05-08

# For data quantity experiment - enough to be useful, not enough to overwhelm the system.
EXTRA_UNLABELED_DATA :=		logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
				logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
				logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00

UNLABELED_TRAINING_DATA :=	logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17 \
				logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
				logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15 \
				logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
				logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
				logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
				logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
				logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
				logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
				logs/2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40
#				logs/2010-10-31_mostly_cars/run1-10-31-2010_17-09-21 \
#				logs/2010-10-31_mostly_cars/run2-10-31-2010_17-18-52 \
#				logs/2010-10-31_mostly_cars/run3-10-31-2010_17-26-53 \
#				logs/2010-10-31_mostly_cars/run4-10-31-2010_17-34-01 \
#				logs/2010-10-31_mostly_cars/run5-10-31-2010_17-39-25 \
#				logs/2010-10-31_mostly_cars/run6-10-31-2010_17-52-08 \
#				logs/2010-10-31_mostly_cars/run7-10-31-2010_17-58-23

UNLABELED_TRAINING_DATA_MEDIUM :=	logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
					logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
					logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
					logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06


LABELED_TRAINING_DATA_TINY := natural/lasuen02-11-17-2009_17-07-43
UNLABELED_TRAINING_DATA_TINY := logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17
BACKGROUND_TINY := 	background/null20-11-16-2009_21-12-04 \
			background/null02-11-16-2009_20-22-34 \
			background/null06-11-16-2009_20-30-22 \
			background/null09-11-16-2009_20-33-22


# -- This is where to set how much data to use.
# Huge.
#BACKGROUND := $(BACKGROUND_FULL)
#TRAINING_DATA := $(LABELED_TRAINING_DATA) $(UNLABELED_TRAINING_DATA)
#TM_PATHS := 	$(addprefix unlabeled_tracks/, $(addsuffix .tm, $(UNLABELED_TRAINING_DATA))) \
#		$(addprefix stanford_track_collection/, $(addsuffix .tm, $(LABELED_TRAINING_DATA)))

# STC-only.
BACKGROUND := $(BACKGROUND_FULL)
TRAINING_DATA := $(LABELED_TRAINING_DATA)
#TM_PATHS := 	$(addprefix stanford_track_collection/, $(addsuffix .tm, $(LABELED_TRAINING_DATA)))

# Medium test with only unlabeled data.
#BACKGROUND := $(BACKGROUND_FULL)
#TRAINING_DATA := $(UNLABELED_TRAINING_DATA)
#TRAINING_DATA := $(UNLABELED_TRAINING_DATA_MEDIUM)
#TM_PATHS := 	$(addprefix stanford_track_collection/, $(addsuffix .tm, $(LABELED_TRAINING_DATA)))

# Tiny, for code testing.
#BACKGROUND := $(BACKGROUND_TINY)
#TRAINING_DATA := $(LABELED_TRAINING_DATA_TINY) $(UNLABELED_TRAINING_DATA_TINY)
#TM_PATHS := 	$(addprefix unlabeled_tracks/, $(addsuffix .tm, $(UNLABELED_TRAINING_DATA_TINY))) \
#		$(addprefix stanford_track_collection/, $(addsuffix .tm, $(LABELED_TRAINING_DATA_TINY)))



TM_DIR := stanford_track_collection

SEED_LABEL_SOURCE := $(TM_DIR)/natural/intersection-04-07-2010_09-21-53.tm

# Program locations.
BACKGROUND_EXTRACTOR		:= $(RACE_ROOT)/src/program/perception/background_extractor
EVALUATE_FRAME_CLASSIFIER	:= $(RACE_ROOT)/src/program/track_tools/evaluate_frame_classifier
TRACK_TOOLS			:= $(RACE_ROOT)/src/program/track_tools/track_tools
TRACK_DATASET_EXTRACTOR		:= $(RACE_ROOT)/src/program/track_tools/track_dataset_extractor
FRAME_DATASET_EXTRACTOR		:= $(RACE_ROOT)/src/program/track_tools/frame_dataset_extractor
TRACK_STATS			:= $(RACE_ROOT)/src/program/track_tools/track_stats
MULTITOOL			:= $(RACE_ROOT)/src/ros-pkg/multibooster/multitool
SEMISUPERVISED			:= $(RACE_ROOT)/src/program/semisupervised/semisupervised
DATA_QUANTITY_EXPERIMENT	:= $(RACE_ROOT)/src/program/semisupervised/data_quantity_experiment2
EFFORT_EXPERIMENT		:= $(RACE_ROOT)/src/program/semisupervised/effort_experiment
EXTRACT_TRACKS			:= RACE_ROOT=$(RACE_ROOT) python $(RACE_ROOT)/src/program/perception/extract_tracks_silently.py
TRACK_CLASSIFIER_FRAME_ONLY	:= $(RACE_ROOT)/src/program/track_tools/track_classifier_frame_only
TRACK_PROJECTOR_DETAILED	:= DRAW_HULL= DRAW_PTS= DRAW_BACKGROUND= $(RACE_ROOT)/src/program/track_tools/track_projector
SYNTHETIC_EXAMPLE		:= $(RACE_ROOT)/src/program/semisupervised/synthetic_example

# Settings.
STANDARD_TRAINING	:= NCANDIDATES=8 MAX_WCS=1000 rosrun multibooster multitool --train # This must match the settings in semisupervised.cpp.
MAX_NUM_EPOCHS		:= 99
MIN_OBJECTIVE		:= 0.02
CONFIDENCE_THRESH	:= 5
NUM_SEED_LABELS		:= 3

all:
	@echo You should probably be more specific.

########################################
# Latex
########################################

LATEX := pdflatex

IJRR_TEX_TURDS := ijrr2011.aux ijrr2011.bbl ijrr2011.blg ijrr2011.dvi ijrr2011.log ijrr2011.out ijrr2011.ps latex_macros.aux
latex/ijrr2011.pdf: 	latex/process_bib.py \
			latex/ijrr2011.tex \
			latex/latex_macros.tex \
			/home/teichman/.mendeley/paper_refs-RSS2011.bib \
			latex/supplemental.bib \
			ijrr_latex_images 
#			latex_images \


	cd $(@D) && python process_bib.py /home/teichman/.mendeley/paper_refs-RSS2011.bib | grep -v 'isbn = ' | grep -v 'doi = ' | grep -v 'issn = ' | grep -v 'url = ' | sed '/annote = {.*},/d' | sed '/annote/,/},/d' > ijrr2011.bib
	cat $(@D)/supplemental.bib >> $(@D)/ijrr2011.bib
	cd $(@D) && $(LATEX) ijrr2011.tex
	cd $(@D) && bibtex ijrr2011
	cd $(@D) && $(LATEX) ijrr2011.tex
	cd $(@D) && $(LATEX) ijrr2011.tex
	cd $(@D) && rm $(IJRR_TEX_TURDS)


RSS_TEX_TURDS := rss2011.aux rss2011.bbl rss2011.blg rss2011.dvi rss2011.log rss2011.out rss2011.ps latex_macros.aux
latex/rss2011.pdf: 	latex/process_bib.py \
			latex/rss2011.tex \
			latex/latex_macros.tex \
			/home/teichman/.mendeley/paper_refs-RSS2011.bib \
			latex_images \
			latex/supplemental.bib

	cd $(@D) && python process_bib.py /home/teichman/.mendeley/paper_refs-RSS2011.bib | grep -v 'isbn = ' | grep -v 'doi = ' | grep -v 'issn = ' | grep -v 'url = ' | sed '/annote = {.*},/d' | sed '/annote/,/},/d' > rss2011.bib
	cat $(@D)/supplemental.bib >> $(@D)/rss2011.bib
	cd $(@D) && $(LATEX) rss2011.tex
	cd $(@D) && bibtex rss2011
	cd $(@D) && $(LATEX) rss2011.tex
	cd $(@D) && $(LATEX) rss2011.tex
	#dvips rss2011.dvi -o rss2011.ps -t letter -Ppdf -G0
	#ps2pdf rss2011.ps
	cd $(@D) && rm $(RSS_TEX_TURDS)


latex/clean:
	-cd $(@D) && rm rss2011.pdf $(TEX_TURDS)

latex/algorithm.pdf: latex/algorithm.tex latex/ieeeconf.cls
	cd $(@D) && $(LATEX) algorithm.tex
	cd $(@D) && $(LATEX) algorithm.tex

########################################
# Statistics
########################################

track_stats:
	@echo ================================================================================
	@echo = Labeled training data, considered unlabeled by the learning algorithm - STC
	@echo ================================================================================
	@echo
	$(TRACK_STATS) $(addprefix stanford_track_collection/, $(addsuffix .tm, $(LABELED_TRAINING_DATA)))
	@echo
	@echo
	@echo ================================================================================
	@echo = Extra unlabeled data for the data quantity experiment.
	@echo ================================================================================
	@echo
	$(TRACK_STATS) $(addprefix unlabeled_tracks/, $(addsuffix .tm, $(EXTRA_UNLABELED_DATA)))
	@echo
	@echo
	@echo ========================================
	@echo = Actually unlabeled training data
	@echo ========================================
	@echo
	$(TRACK_STATS) $(addprefix unlabeled_tracks/, $(addsuffix .tm, $(UNLABELED_TRAINING_DATA)))


########################################
# Synthetic Data Example
########################################

synthetic_example_output:
	mkdir -p $@
	$(SYNTHETIC_EXAMPLE) $@


########################################
# Data Quantity Experiment
########################################

NUM_PCTS := 10
quantity/done_learning: $(CACHED_BACKGROUND) seed.tm.mbd $(CACHED_TRAINING) $(CACHED_EXTRA_UNLABELED)
	@echo Running data quantity experiment, saving stdout into quantity-log.txt.
	$(DATA_QUANTITY_EXPERIMENT) $(NUM_PCTS) $(@D)/pcts $(MAX_NUM_EPOCHS) $(MIN_OBJECTIVE) $(CONFIDENCE_THRESH) \
	--seed-labels seed.tm.mbd $(CACHED_BACKGROUND) \
	--unlabeled $(CACHED_TRAINING) $(CACHED_EXTRA_UNLABELED) > quantity-log.txt
	mv quantity-log.txt $(@D)
	touch $@

quantity/pcts/accuracy.pdf: quantity/pcts/percents_inducted.txt plot_accuracy_vs_quantity.py semisupervised_common_plotting.py quantity/accuracy_results_computed quantity/pcts/baseline_results.txt
	python plot_accuracy_vs_quantity.py $(@D)

quantity/accuracy_results_computed: quantity/done_learning $(CACHED_TESTING)
	for dir in `find $(@D)/pcts/* -maxdepth 0 -type d`; do \
	echo $$dir/`ls $$dir | grep epoch | awk 'END {print $$0}'`/classifier.mb; \
	$(EVALUATE_FRAME_CLASSIFIER) $$dir/`ls $$dir | grep epoch | awk 'END {print $$0}'`/classifier.mb $(CACHED_TESTING) $$dir/final_results; \
	if [ $$? -ne 0 ]; then break; fi; \
	done;
	touch $@

quantity/pcts/percents_inducted.txt: quantity/done_learning
	rm -f $@
	for dir in `find $(@D)/* -maxdepth 0 -type d`; do \
	cat $$dir/`ls $$dir | grep epoch | awk 'END {print $$0}'`/induction_stats.txt | awk '{if(NR==3) total=$$NF; if(NR==5) ind=$$NF} END {print 100.0 * ind / total}' >> $@; \
	done;

quantity/pcts/max_wcs.txt: quantity/done_learning
	cat `find quantity/pcts/* -name timing.txt` | grep 'weak classifier' | awk '{if($$NF > max) max = $$NF} END {print max}' > $@

# baseline/classifier.mb is trained to have quantity/pcts/max_wcs.txt WCs.
quantity/pcts/baseline_results.txt: baseline/classifier.mb $(CACHED_TESTING)
	mkdir -p $(@D)
	$(EVALUATE_FRAME_CLASSIFIER) baseline/classifier.mb $(CACHED_TESTING) $(basename $@)  # Classifier is trained to baseline/quantity_max_wcs.txt WCs.


########################################
# IJRR Plots
########################################

ijrr_latex_images:	latex/img/overfitting_test.pdf \
			latex/img/synthetic_example_output

latex/img/overfitting_test.pdf: baseline/overfitting_test.pdf
	mkdir -p $(@D)
	cp $^ $@

latex/img/synthetic_example_output: synthetic_example_output
	cd latex/img && ln -s ../../$< 

########################################
# Special Plots
########################################

latex_images: 	latex/img/ssl_vs_cssl.pdf \
		latex/img/ssl_induction.pdf \
		latex/img/frame_induction.pdf \
		latex/img/ssl_accuracy.pdf \
		latex/img/ssl_confusion_matrix.pdf \
		latex/img/baseline_confusion_matrix.pdf \
		latex/img/accuracy_vs_quantity.pdf \
		latex/ssl_baseline_accuracy.tex \
		latex/ssl_accuracy.tex \
		latex/quantity_baseline_accuracy.tex

latex/abstract.txt: latex/rss2011.tex
	cat $< | sed -n '/abstract/,/abstract/p' | grep -v 'begin{abstract}' | grep -v 'end{abstract}' > $@

latex/ssl_baseline_accuracy.tex: ssl/baseline_results.txt
	cat $< | grep 'Total acc' | awk '{printf("%.1f\\%\\xspace", $$NF * 100)}' > $@

latex/quantity_baseline_accuracy.tex: quantity/pcts/baseline_results.txt
	cat $< | grep 'Total acc' | awk '{printf("%.1f\\%\\xspace", $$NF * 100)}' > $@

latex/ssl_accuracy.tex: ssl/accuracy_results_computed
	cat ssl/`ls ssl | grep epoch | awk 'END {print $$0}'`/results.txt | grep 'Total acc' | awk '{printf("%.1f\\%\\xspace", $$NF * 100)}' > $@

latex/frame_accuracy.tex: frame/accuracy_results_computed
	cat frame/`ls frame | grep epoch | awk 'END {print $$0}'`/results.txt | grep 'Total acc' | awk '{printf("%.1f\\%\\xspace", $$NF * 100)}' > $@

latex/cssl-rr_accuracy.tex: cssl-rr/accuracy_results_computed
	cat cssl-rr/`ls cssl-rr | grep epoch | awk 'END {print $$0}'`/results.txt | grep 'Total acc' | awk '{printf("%.1f\\%\\xspace", $$NF * 100)}' > $@


latex/img/ssl_vs_cssl.pdf: 	ssl/accuracy_results_computed \
				cssl/accuracy_results_computed \
				cssl-rr/accuracy_results_computed \
				plot_ssl_vs_cssl.py \
				semisupervised_common_plotting.py
	mkdir -p $(@D)
	python plot_ssl_vs_cssl.py $(@D) # Should this have been generalized?

latex/img/ssl_induction.pdf: ssl/induction.pdf
	mkdir -p $(@D)
	cp ssl/induction.pdf $@

latex/img/frame_induction.pdf: plot_frame_induction.py semisupervised_common_plotting.py frame/done_learning
	mkdir -p $(@D)
	python $<
	mv frame_induction.pdf frame_induction.png $(@D)

latex/img/ssl_accuracy.pdf: ssl/accuracy.pdf
	mkdir -p $(@D)
	cp $^ $@

latex/img/ssl_confusion_matrix.pdf: ssl/accuracy_results_computed
	mkdir -p $(@D)
	cp ssl/`ls ssl | grep epoch | awk 'END {print $$0}'`/results.pdf $@

latex/img/baseline_confusion_matrix.pdf: ssl/baseline_results.txt
	mkdir -p $(@D)
	cp ssl/baseline_results.pdf $@

latex/img/accuracy_vs_quantity.pdf: quantity/pcts/accuracy.pdf
	mkdir -p $(@D)
	cp $< $@

########################################
# Visualization
########################################

visualization:	videos/logs/2010-10-31_mostly_cars/run2-10-31-2010_17-18-52-detailed.avi \
		videos/logs/2010-10-31_mostly_cars/run6-10-31-2010_17-52-08-detailed.avi \
		videos/logs/2010-10-31_mostly_cars/run3-10-31-2010_17-26-53-detailed.avi

videos/%-detailed.avi: classified_tracks/%.tm %.llf
	mkdir -p $(@D)
	$(TRACK_PROJECTOR_DETAILED) $^ $@.images
	cd $@.images && mencoder "mf://*.png" -mf fps=15 -o test.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=3200 -vf eq2=1.0:1.3:0.2:1.0,scale=616:808
	mv $@.images/test.avi $@

classified_tracks/%.tm: unlabeled_tracks/%.tm cssl-rr/done_learning
	mkdir -p $(@D)
	$(TRACK_CLASSIFIER_FRAME_ONLY) cssl-rr/`ls cssl-rr | grep epoch | awk 'END {print $$0}'`/classifier.mb $< $@


########################################
# Semisupervised Learning
########################################

CACHED_BACKGROUND := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(BACKGROUND)))
CACHED_TRAINING := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(TRAINING_DATA)))
CACHED_TRAINING_UNLABELED := $(addprefix cached_descriptors/forcibly_unlabeled/, $(addsuffix .tm.mbd, $(TRAINING_DATA)))
CACHED_TESTING := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(TESTING_DATA)))
CACHED_EXTRA_UNLABELED := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(EXTRA_UNLABELED_DATA)))
CACHED_ALL := $(CACHED_BACKGROUND) $(CACHED_TRAINING) $(CACHED_TESTING) $(CACHED_EXTRA_UNLABELED)

SSL_TYPES := ssl cssl-rr cssl-rrp frame cssl # cssl-rb

all_ssl_runs: $(SSL_TYPES)

all_ssl_runs_without_plotting: $(addsuffix /accuracy_results_computed, $(SSL_TYPES))


define SSL_EXPERIMENT_RULE_GENERATOR
$(1): $(1)/accuracy.pdf $(1)/induction.pdf $(1)/timing.pdf

$(1)/done_learning: $(CACHED_BACKGROUND) seed.tm.mbd $(CACHED_TRAINING)
	@echo Running semisupervised learning, saving stdout into $(1)-log.txt.
	RANDOM_SEED=-1 $(SEMISUPERVISED) $(1) $(1) $(MAX_NUM_EPOCHS) $(MIN_OBJECTIVE) $(CONFIDENCE_THRESH) \
	--seed-labels seed.tm.mbd $(CACHED_BACKGROUND) \
	--unlabeled $(CACHED_TRAINING) > $(1)-log.txt
	mv $(1)-log.txt $(1)/
	touch $$@

$(1)/max_wcs.txt: $(1)/done_learning
	cat `find $(1)/* -name timing.txt` | grep 'weak classifier' | awk '{if($$$$NF > max) max = $$$$NF} END {print max}' > $$@

$(1)/baseline_results.txt: baseline/classifier.mb $(CACHED_TESTING) $(1)/max_wcs.txt
	WC_LIMIT=`echo $(1)/max_wcs.txt` $(EVALUATE_FRAME_CLASSIFIER) baseline/classifier.mb $(CACHED_TESTING) $$(basename $$@)

$(1)/induction.pdf: plot_induction.py semisupervised_common_plotting.py $(1)/done_learning
	python plot_induction.py $(1)

$(1)/timing.pdf: plot_timing.py semisupervised_common_plotting.py $(1)/done_learning
	python plot_timing.py $(1)

$(1)/accuracy.pdf: $(1)/accuracy_results_computed plot_accuracy.py semisupervised_common_plotting.py $(1)/baseline_results.txt
	python plot_accuracy.py $(1)

$(1)/accuracy_results_computed: $(1)/done_learning $(CACHED_TESTING)
	for dir in `ls $(1) | egrep epoch[0-9]*`; do \
	echo $(1)/$$$$dir/classifier.mb; \
	$(EVALUATE_FRAME_CLASSIFIER) $(1)/$$$$dir/classifier.mb $(CACHED_TESTING) $(1)/$$$$dir/results; \
	if [ $$$$? -ne 0 ]; then break; fi; \
	done;
	touch $$@
endef
$(foreach VAL,$(SSL_TYPES),$(eval $(call SSL_EXPERIMENT_RULE_GENERATOR,$(VAL))))


# -- Sanity check to make sure that this works when using only unlabeled data.

cssl-rr_completely_unlabeled/done_learning: $(CACHED_BACKGROUND) seed.tm.mbd $(CACHED_TRAINING_UNLABELED)
	@echo Running semisupervised learning, saving stdout into cssl-rr_completely_unlabeled-log.txt.
	RANDOM_SEED=-1 $(SEMISUPERVISED) cssl-rr cssl-rr_completely_unlabeled $(MAX_NUM_EPOCHS) $(MIN_OBJECTIVE) $(CONFIDENCE_THRESH) \
	--seed-labels seed.tm.mbd $(CACHED_BACKGROUND) \
	--unlabeled $(CACHED_TRAINING_UNLABELED) > cssl-rr_completely_unlabeled-log.txt
	mv cssl-rr_completely_unlabeled-log.txt cssl-rr_completely_unlabeled/
	touch $@

cssl-rr_completely_unlabeled/accuracy_results_computed: cssl-rr_completely_unlabeled/done_learning $(CACHED_TESTING)
	for dir in `ls cssl-rr_completely_unlabeled | egrep epoch[0-9]*`; do \
	echo cssl-rr_completely_unlabeled/$$dir/classifier.mb; \
	$(EVALUATE_FRAME_CLASSIFIER) cssl-rr_completely_unlabeled/$$dir/classifier.mb $(CACHED_TESTING) cssl-rr_completely_unlabeled/$$dir/results; \
	if [ $$? -ne 0 ]; then break; fi; \
	done;
	touch $@


#CACHED_UNLABELED := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(UNLABELED_TRAINING_DATA_MEDIUM)))
#cssl-rr-plus/done_learning: $(CACHED_BACKGROUND) seed.tm.mbd $(CACHED_TRAINING) $(CACHED_UNLABELED)
#	@echo Running semisupervised learning, saving stdout into $(@D)-log.txt.
#	$(SEMISUPERVISED) cssl-rr $(@D) $(MAX_NUM_EPOCHS) $(MIN_OBJECTIVE) $(CONFIDENCE_THRESH) \
#	--seed-labels seed.tm.mbd $(CACHED_BACKGROUND) \
#	--unlabeled $(CACHED_TRAINING) $(CACHED_UNLABELED) > $(@D)-log.txt
#	mv $(@D)-log.txt $(@D)/
#	touch $@
#
#cssl-rr-plus/induction.pdf: plot_induction.py semisupervised_common_plotting.py cssl-rr-plus/done_learning
#	python plot_induction.py cssl-rr-plus
#
#cssl-rr-plus/timing.pdf: plot_timing.py semisupervised_common_plotting.py cssl-rr-plus/done_learning
#	python plot_timing.py cssl-rr-plus
#
#cssl-rr-plus/accuracy.pdf: cssl-rr-plus/accuracy_results_computed plot_accuracy.py semisupervised_common_plotting.py baseline/results_vs_ssl.txt
#	python plot_accuracy.py cssl-rr-plus
#
#cssl-rr-plus/accuracy_results_computed: cssl-rr-plus/done_learning $(CACHED_TESTING)
#	for dir in `ls cssl-rr-plus | egrep epoch[0-9]*`; do \
#	echo cssl-rr-plus/$$dir/classifier.mb; \
#	$(EVALUATE_FRAME_CLASSIFIER) cssl-rr-plus/$$dir/classifier.mb $(CACHED_TESTING) cssl-rr-plus/$$dir/results.txt; \
#	if [ $$? -ne 0 ]; then break; fi; \
#	done;
#	touch $@


########################################
# Baseline classifier training
########################################

all_baseline: baseline/results baseline/overfitting_test

# ; \ are necessary or the exports won't take.
baseline/classifier.mb: baseline/train.mbd quantity/pcts/max_wcs.txt
	export MAX_WCS=$$(echo `cat quantity/pcts/max_wcs.txt`); \
	export NCANDIDATES=8; \
	echo Using $$MAX_WCS weak classifiers, $$NCANDIDATES candidates. > $(@D)/training_params.txt; \
	rosrun multibooster multitool --train $< $@

baseline/train.mbd: $(CACHED_BACKGROUND) $(CACHED_TRAINING)
	mkdir -p $(@D)
	$(TRACK_TOOLS) --join-tm-mbd --no-unlabeled $^ $@

baseline/overfitting_test: baseline/classifier.mb $(CACHED_TESTING)
	mkdir -p $@
	for val in $$(seq 25 25 225); do echo WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; done
	for val in $$(seq 250 250 1750); do echo WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; done
	for val in $$(seq 2000 4000 $$(cat quantity/pcts/max_wcs.txt)); do echo WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $^ $@/$$(printf "%05d" $$val)-wcs; done

baseline/overfitting_test.pdf: baseline/overfitting_test plot_overfitting_test.py
	python plot_overfitting_test.py baseline

########################################
# Effort Experiment
########################################

# EFFORT_UNLABELED_TRACKS :=	logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17 \
# 				logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
# 				logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15 \
# 				logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
# 				logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
# 				logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
# 				logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
# 				logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
# 				logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
# 				logs/2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40

#EFFORT_UNLABELED_TRACKS := $(EXTRA_UNLABELED_DATA)

#EFFORT_UNLABELED_TRACKS :=	logs/2011-04-07_hoover/overhead1-04-07-2011_12-55-40 \
#				logs/2011-04-07_hoover/overhead2-04-07-2011_13-09-21 \
#				logs/2011-04-07_hoover/overhead3-04-07-2011_13-23-01 \
#				logs/2011-04-07_hoover/overhead4-04-07-2011_13-39-58 \
#				logs/2011-04-07_hoover/overhead5-04-07-2011_13-53-33 \
#				logs/2011-04-07_hoover/overhead6-04-07-2011_14-07-56 \
#				logs/2011-04-07_hoover/overhead7-04-07-2011_14-21-30 \
#				logs/2011-04-07_hoover/overhead8-04-07-2011_15-10-29 \
#				logs/2011-04-07_hoover/overhead9-04-07-2011_15-24-04 \
#				logs/2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17 \
#				logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
#				logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15 \
#				logs/2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
#				logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
#				logs/2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
#				logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
#				logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
#				logs/2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
#				logs/2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40


EFFORT_UNLABELED_TRACKS :=	logs/2011-04-07_hoover/overhead1-04-07-2011_12-55-40 \
				logs/2011-04-07_hoover/overhead2-04-07-2011_13-09-21 \
				logs/2011-04-07_hoover/overhead4-04-07-2011_13-39-58 \
				logs/2011-04-07_hoover/overhead6-04-07-2011_14-07-56 \
				logs/2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
				logs/2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
				logs/2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
				logs/2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
				logs/2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15

EFFORT_PCTS := 0.02 0.05 0.1 0.2 0.5 1.0
EFFORT_BACKGROUND := $(CACHED_BACKGROUND)
EFFORT_HAND_LABELED := seed.tm.mbd $(CACHED_TRAINING)
EFFORT_UNLABELED := $(addprefix cached_descriptors/, $(addsuffix .tm.mbd, $(EFFORT_UNLABELED_TRACKS)))

workspace/effort_experiment/effort.pdf:	plot_effort.py \
					workspace/effort_experiment/supervised_results_computed \
					workspace/effort_experiment/semisupervised_results_computed \
					workspace/effort_experiment/testset_stats.txt
	python plot_effort.py $(@D)

workspace/effort_experiment/results_computed: 	workspace/effort_experiment/supervised_results_computed \
						workspace/effort_experiment/semisupervised_results_computed

workspace/effort_experiment/track_stats: $(addprefix unlabeled_tracks/, $(addsuffix .tm, $(EFFORT_UNLABELED_TRACKS)))
	$(TRACK_STATS) $^

workspace/effort_experiment/testset_stats.txt: $(addprefix stanford_track_collection/, $(addsuffix .tm, $(TESTING_DATA)))
	$(TRACK_STATS) $^ > $@

workspace/effort_experiment/supervised_results_computed: workspace/effort_experiment/done_learning $(CACHED_TESTING)
	for dir in `find workspace/effort_experiment/* -type d | grep 'pct/supervised'`; do \
	if [ -e $$dir/classifier.mb ]; then \
	$(EVALUATE_FRAME_CLASSIFIER) $$dir/classifier.mb $(CACHED_TESTING) $$dir/results; \
	max_wcs=$$(rosrun multibooster multitool --status $$(find $$dir/../ -name '*epoch*' -type d | sort | tail -n1)/classifier.mb | grep 'Number of weak classifiers' | awk '{print $$NF}'); \
	echo Max WCs is $$max_wcs; \
	for val in `seq 200 4000 $$max_wcs`; do \
	mkdir -p $$dir/overfit_test; \
	echo WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $$dir/classifier.mb $(CACHED_TESTING) $$dir/overfit_test/$$(printf "%05d" $$val)-wcs; \
	WC_LIMIT=$$val $(EVALUATE_FRAME_CLASSIFIER) $$dir/classifier.mb $(CACHED_TESTING) $$dir/overfit_test/$$(printf "%05d" $$val)-wcs; \
	done; \
	fi; \
	done;

workspace/effort_experiment/semisupervised_results_computed: workspace/effort_experiment/done_learning
	for dir in `find workspace/effort_experiment/* -maxdepth 0 -type d`; do \
	path=`find $$dir/cssl-rr/* -name classifier.mb | sort | tail -n1`; \
	if [ -n "$$path" ]; then \
	echo Evaluating classifier: $$path; \
	$(EVALUATE_FRAME_CLASSIFIER) $$path $(CACHED_TESTING) $$dir/final_results; \
	fi; \
	done

workspace/effort_experiment/done_learning: workspace/effort_params.txt $(EFFORT_UNLABELED) $(EFFORT_BACKGROUND) $(EFFORT_HAND_LABELED)
	$(EFFORT_EXPERIMENT) $<
	touch $@

workspace/effort_params.txt:
	@echo workspace_path_ workspace/effort_experiment > $@
	@echo >> $@
	@echo hand_labeled_percents_ >> $@
	@echo $(EFFORT_PCTS) | xargs -n1 | wc -l >> $@
	@echo $(EFFORT_PCTS) | xargs -n1 >> $@
	@echo >> $@
	@echo max_num_epochs_ $(MAX_NUM_EPOCHS) >> $@
	@echo confidence_thresh_ $(CONFIDENCE_THRESH) >> $@
	@echo min_objective_ $(MIN_OBJECTIVE) >> $@
	@echo >> $@
	@echo background_paths_ >> $@
	@echo $(EFFORT_BACKGROUND) | xargs -n1 | wc -l >> $@
	@echo $(EFFORT_BACKGROUND) | xargs -n1 >> $@
	@echo >> $@
	@echo hand_labeled_paths_ >> $@
	@echo $(EFFORT_HAND_LABELED) | xargs -n1 | wc -l >> $@
	@echo $(EFFORT_HAND_LABELED) | xargs -n1 >> $@
	@echo >> $@
	@echo unlabeled_paths_ >> $@
	@echo $(EFFORT_UNLABELED) | xargs -n1 | wc -l >> $@
	@echo $(EFFORT_UNLABELED) | xargs -n1 >> $@


########################################
# Dataset Setup
########################################

all_caching:	$(CACHED_ALL)

cached_descriptors/%.tm.mbd: $(TM_DIR)/%.tm
	mkdir -p $(@D)
	@echo Using $$NUM_THREADS threads.
	$(TRACK_TOOLS) --cache-descriptors $^ $@

cached_descriptors/forcibly_unlabeled/%.tm.mbd: cached_descriptors/%.tm.mbd
	mkdir -p $(@D)
	$(TRACK_TOOLS) --label-all-cached -2 $^ $@

cached_descriptors/%.tm.mbd: unlabeled_tracks/%.tm
	mkdir -p $(@D)
	@echo Using $$NUM_THREADS threads.
	$(TRACK_TOOLS) --cache-descriptors $^ $@

test.tm: $(addprefix $(TM_DIR)/, $(addsuffix .tm, $(TESTING_DATA)))
	$(TRACK_TOOLS) --join $@ $^

#seed.tm.mbd: $(SEED_LABEL_SOURCE)
#	$(TRACK_TOOLS) --extract-seed-labels $(NUM_SEED_LABELS) $^ seed.tm unlabeled.tm
#	rm unlabeled.tm
#	$(TRACK_TOOLS) --cache-descriptors seed.tm $@

seed.tm.mbd: nine_hand_chosen_tracks.tm
	$(TRACK_TOOLS) --cache-descriptors $< $@

all_unlabeled_tracks: $(addprefix unlabeled_tracks/, $(addsuffix .tm, $(UNLABELED_TRAINING_DATA)))
unlabeled_tracks/%.tm: %.log.gz
	mkdir -p $(@D)
	$(EXTRACT_TRACKS) $^ $@

############################################################
# Log Preparation
############################################################

all_indices: $(addsuffix .vlf.index.gz, $(UNLABELED_TRAINING_DATA))

logs/%.vlf.index.gz: logs/%.log.gz logs/%.vlf
	$(VLF_INDEX) $^

