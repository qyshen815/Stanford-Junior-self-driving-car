############################################################
# To repeat these experiments...
############################################################
# logfiles/ should contain all necessary vlfs.
# background_tracks/ should contain all the background extracted tracks in STC.
# labeled_tracks/ should contain all the 'natural' extracted tracks in STC.
# tex/static_imagery should point to ~/sync/media/2010/10-09-09_ICRA2011_static_imagery

############################################################
# Global settings
############################################################

RACE_ROOT := ../../../
export NUM_THREADS:=40
export ROS_PACKAGE_PATH:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

.SECONDEXPANSION: # Allows use of automatic vars in prerequisites.

############################################################
# Programs
############################################################

TRACK_DATASET_EXTRACTOR := $(RACE_ROOT)/src/program/track_tools/track_dataset_extractor
FRAME_DATASET_EXTRACTOR := $(RACE_ROOT)/src/program/track_tools/frame_dataset_extractor
EVALUATE_FRAME_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/evaluate_frame_classifier
EVALUATE_TRACK_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/evaluate_track_classifier
EVALUATE_COMBINED_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/evaluate_combined_classifier
EXTRACT_TRACKS := RACE_ROOT=$(RACE_ROOT) python $(RACE_ROOT)/src/program/perception/extract_tracks_silently.py
TRAIN_FRAME_CLASSIFIER := NCANDIDATES=100 MAX_WCS=1000 rosrun multibooster multitool --train
TRAIN_TRACK_CLASSIFIER := NCANDIDATES=100 MAX_WCS=100 rosrun multibooster multitool --train
TRAIN_COMBINED_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/train_combined_classifier
TRACK_STATS := $(RACE_ROOT)/src/program/track_tools/track_tools --status
TRACK_PLOTS := $(RACE_ROOT)/src/program/track_tools/track_plots
TRACK_TOOLS := $(RACE_ROOT)/src/program/track_tools/track_tools
PARSE_TO_KML := python kml_generation/parseToKML.py

############################################################
# Data
############################################################

NEGATIVES := 		null20-11-16-2009_21-12-04 \
			null02-11-16-2009_20-22-34 \
			null06-11-16-2009_20-30-22 \
			null09-11-16-2009_20-33-22 \
			null10-11-16-2009_20-40-17 \
			null15-11-16-2009_21-05-30 \
			null16-11-16-2009_21-07-32 \
			null17-11-16-2009_21-09-21 \
			null18-11-16-2009_21-10-09

LOGFILES :=	 	white_plaza01-11-17-2009_17-35-07 \
			white_plaza02-11-17-2009_17-35-42 \
			white_plaza03-11-17-2009_17-47-49 \
			white_plaza04-11-17-2009_17-49-05 \
			white_plaza05-11-17-2009_17-59-14 \
			white_plaza06-11-17-2009_18-13-19 \
			lasuen01-11-17-2009_16-56-40 \
			lasuen02-11-17-2009_17-07-43 \
			lasuen03-11-17-2009_17-15-00 \
			lasuen04-11-17-2009_17-18-05 \
			lasuen05-11-17-2009_17-20-16 \
			lasuen06-11-17-2009_17-26-37 \
			dish_area01-11-17-2009_20-08-30 \
			dish_area02-11-17-2009_20-13-49 \
			driving_campus_loop-11-17-2009_20-22-00 \
			driving_test_set-11-17-2009_19-13-35 \
			el_camino01-11-17-2009_19-17-25 \
			el_camino02-11-17-2009_19-21-11 \
			el_camino_and_cambridge-11-17-2009_19-32-08 \
			lasuen01-11-17-2009_16-56-40 \
			lasuen02-11-17-2009_17-07-43 \
			lasuen03-11-17-2009_17-15-00 \
			lasuen04-11-17-2009_17-18-05 \
			lasuen05-11-17-2009_17-20-16 \
			lasuen06-11-17-2009_17-26-37 \
			lomita_and_santa_teresa01-11-17-2009_18-35-12 \
			oval01-11-17-2009_18-52-00 \
			page_mill_and_hansen01-11-17-2009_19-49-18 \
			page_mill_and_hansen02-11-17-2009_19-56-57 \
			page_mill_and_ramos-11-17-2009_19-47-17 \
			page_mill_and_soccer02-11-17-2009_19-43-22 \
			serra_and_galvez-11-17-2009_19-06-10 \
			intersection-04-07-2010_09-05-08 \
			intersection-04-07-2010_09-21-53 \
			pass2-10-16-2009_15-00-46 \
			CampusLoop3-10-03-2009_15-32-37 \
			ungrad_area-10-06-2010_16-12-40-cut

TEST_DATA := 		dish_area02-11-17-2009_20-13-49 \
			page_mill_and_hansen01-11-17-2009_19-49-18 \
			page_mill_and_hansen02-11-17-2009_19-56-57 \
			white_plaza02-11-17-2009_17-35-42 \
			driving_test_set-11-17-2009_19-13-35

TRAINING_DATA := 	lasuen02-11-17-2009_17-07-43 \
			CampusLoop3-10-03-2009_15-32-37-FirstMinute \
			pass2-10-16-2009_15-00-46 \
			lomita_and_santa_teresa01-11-17-2009_18-35-12 \
			intersection-04-07-2010_09-05-08 \
			intersection-04-07-2010_09-21-53

# These are beyond the STC.  ADDITIONAL_TRAINING_TMS should be empty for published experiments.
#FOO := sign hoover
#ADDITIONAL_TRAINING_TMS :=
ADDITIONAL_TRAINING_TMS := sign hoover

HOLDOUT_DATA :=	ungrad_area-10-06-2010_16-12-40-cut


############################################################
# Everything
############################################################

foo:
	@echo You should probably be more specific.

all: 	plots/testset_purehistogram_frame.pdf \
	plots/stc_table.tex \
	results_mll/results_mll \
	results_mel/results_mel \
	results_no_holistic/results_no_holistic

############################################################
# Statistics
############################################################

TRAINING_VLFS := $(addprefix logfiles/, $(addsuffix .vlf, $(TRAINING_DATA) $(HOLDOUT_DATA)))
NEGATIVES_VLFS := $(addprefix logfiles/, $(addsuffix .vlf, $(NEGATIVES)))
TEST_VLFS :=  $(addprefix logfiles/, $(addsuffix .vlf, $(TEST_DATA)))
log_stats:
	@echo Minutes of training data:
	du -Lcs $(TRAINING_VLFS) | grep total | awk '{print $$1/3500/60}' # 3.5MB / second from the velodyne.
	@echo
	@echo Minutes of extra negatives data:
	du -Lcs $(NEGATIVES_VLFS) | grep total | awk '{print $$1/3500/60}'
	@echo
	@echo Minutes of test data:
	du -Lcs $(TEST_VLFS) | grep total | awk '{print $$1/3500/60}'

plots/testset_purehistogram_frame.pdf: $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	mkdir -p $(@D)
	$(TRACK_PLOTS) $^ plots/testset

track_stats:
	@echo ========================================
	@echo = Test Data
	@echo ========================================
	@echo
	@$(TRACK_STATS) $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	@echo
	@echo
	@echo ========================================
	@echo = Training Data, no holdout
	@echo ========================================
	@echo
	$(TRACK_STATS) $(addprefix labeled_tracks/, $(addsuffix .tm, $(TRAINING_DATA))) $(addprefix background_tracks/, $(addsuffix .tm, $(NEGATIVES)))
	@echo
	@echo
	@echo ========================================
	@echo = Holdout set only
	@echo ========================================
	@echo
	@$(TRACK_STATS) $(addprefix labeled_tracks/, $(addsuffix .tm, $(HOLDOUT_DATA)))
	@echo
	@echo
	@echo ========================================
	@echo = Grand Total
	@echo ========================================
	@$(TRACK_STATS) $(addprefix background_tracks/, $(addsuffix .tm, $(NEGATIVES))) \
			$(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA) $(TRAINING_DATA) $(HOLDOUT_DATA)))

plots/stc_table.tex:
	mkdir -p $(@D)
	$(TRACK_TOOLS) --latex-table --training $(addprefix labeled_tracks/, $(addsuffix .tm, $(TRAINING_DATA) $(HOLDOUT_DATA))) $(addprefix background_tracks/, $(addsuffix .tm, $(NEGATIVES))) --testing $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA))) | grep '\\begin{tabular}' -A999 > $@

all_kml: $(addprefix kml/, $(addsuffix .log.gz.kml, $(LOGFILES)))

kml/%.log.gz.kml: logfiles/%.log.gz
	mkdir -p $(@D)
	$(PARSE_TO_KML) $^
	mv $^.kml $@


############################################################
# General Evaluation
############################################################

results: results_mll/results_mll results_mel/results_mel results_no_holistic/results_no_holistic

# Use the full classifier with the weights that were trained on the holdout set.
results_mll/results_mll: frame_classifier_full.mb track_classifier.mb weights $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	mkdir -p $(@D)
	$(EVALUATE_COMBINED_CLASSIFIER) $^ $@ > evaluation_log_mll.txt
	touch $@
	#cat evaluation_log_mll.txt | grep 'Frame:' | awk '{sum += $$2} END {print sum / NR}' > $@/results_mll_speeds.txt
	#cat evaluation_log_mll.txt | grep 'Global (am' | awk '{sum += $$3} END {print sum / NR}' >> $@/results_mll_speeds.txt

results_mel/results_mel: frame_classifier_full.mb track_classifier.mb weights_mel $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	mkdir -p $(@D)
	$(EVALUATE_COMBINED_CLASSIFIER) $^ $@
	touch $@

results_no_holistic/results_no_holistic: frame_classifier_full.mb track_classifier.mb weights_no_holistic $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	mkdir -p $(@D)
	$(EVALUATE_COMBINED_CLASSIFIER) $^ $@
	touch $@

training_set_performance/track_results: track_classifier.mb $(addprefix labeled_tracks/, $(addsuffix .tm, $(TRAINING_DATA))) $(addsuffix .tm, $(addprefix background_tracks/, $(NEGATIVES)))
	mkdir -p $(@D)
	$(EVALUATE_TRACK_CLASSIFIER) $^ $@


############################################################
# Statisticial significance test
############################################################

# If this doesn't work, you aren't using the right shell.  (Use bash.)
random:
	@echo A random number: $$RANDOM.

# You must be using /bin/bash for this to work: {0..9} and $RANDOM.
significance/runs_completed:
	mkdir -p $(@D)
	for num in {0..9}; do \
	echo "Starting run $$num"; \
	DIR=$(@D)/run`printf "%02d" $$num`; \
	echo "Making dir $$DIR".; \
	mkdir -p $$DIR; \
	export SRAND=$$RANDOM; \
	echo "Using random seed $$SRAND".; \
	echo $$SRAND > $$DIR/seed.txt; \
	make results; \
	make orienter_experiment; \
	echo "Cleaning up."; \
	rm -rf orienter_experiment/with/frame_mbds           orienter_experiment/without/frame_mbds; \
	rm -rf orienter_experiment/with/frame_classifier.mbd orienter_experiment/without/frame_classifier.mbd; \
	mv evaluation_log_mll.txt *.mb weights* results_mll results_mel results_no_holistic orienter_experiment $$DIR; \
	rm -rf frame_classifier_adbf.mbd frame_classifier_full.mbd frame_mbds track_mbds track_classifier.mbd; \
	rm `find $$DIR/* -name '*all_misclassified.tm'`; \
	done;
	touch $@


############################################################
# Point cloud orienter experiment
############################################################

orienter_experiment: orienter_experiment/without/results.txt orienter_experiment/with/results.txt

CASES := with without

define ORIENTER_EXPERIMENT_GENERATOR

orienter_experiment/$(1)/results.txt: orienter_experiment/$(1)/frame_classifier.mb $(addprefix labeled_tracks/, $(addsuffix .tm, $(TEST_DATA)))
	NAME=$$@; ORIENTER=$(1) $(EVALUATE_FRAME_CLASSIFIER) $$^ $$$${NAME%.txt}

orienter_experiment/$(1)/frame_classifier.mb: orienter_experiment/$(1)/frame_classifier.mbd
	$(TRAIN_FRAME_CLASSIFIER) $$^ $$@

orienter_experiment/$(1)/frame_classifier.mbd: $$(addprefix orienter_experiment/$(1)/frame_mbds/decimated/, $(addsuffix .mbd, $(NEGATIVES) $(TRAINING_DATA) $(HOLDOUT_DATA)))
	mkdir -p $$(@D)
	rosrun multibooster multitool --join $$^ $$@

orienter_experiment/$(1)/frame_mbds/decimated/%.mbd: orienter_experiment/$(1)/frame_mbds/%.mbd
	mkdir -p $$(@D)
	rosrun multibooster multitool --decimateDataset 0.2 $$^ $$@

.SECONDARY: $$(addprefix orienter_experiment/$(1)/frame_mbds/, $(addsuffix .mbd, $(TRAINING_DATA) $(NEGATIVES) $(HOLDOUT_DATA))) # Don't delete the un-decimated datasets.

orienter_experiment/$(1)/frame_mbds/%.mbd: labeled_tracks/%.tm
	mkdir -p $$(@D)
	ORIENTER=$(1) $(FRAME_DATASET_EXTRACTOR) $$^ $$@

orienter_experiment/$(1)/frame_mbds/%.mbd: background_tracks/%.tm
	mkdir -p $$(@D)
	ORIENTER=$(1) $(FRAME_DATASET_EXTRACTOR) $$^ $$@

endef
$(foreach VAL,$(CASES),$(eval $(call ORIENTER_EXPERIMENT_GENERATOR,$(VAL))))


############################################################
# Classifier Training
############################################################

classifier: weights.eig frame_classifier_full.mb track_classifier.mb

weights.eig: weights
	rosrun eigen_extensions convert $< $@

weights:	frame_classifier_adbf.mb track_classifier.mb \
		$(addprefix labeled_tracks/, $(addsuffix .tm, $(HOLDOUT_DATA)))
	$(TRAIN_COMBINED_CLASSIFIER) $^ $@

frame_classifier_adbf.mb: frame_classifier_adbf.mbd
	$(TRAIN_FRAME_CLASSIFIER) $^ $@

frame_classifier_full.mb: frame_classifier_full.mbd
	$(TRAIN_FRAME_CLASSIFIER) $^ $@

track_classifier.mb: track_classifier.mbd
	$(TRAIN_TRACK_CLASSIFIER) $^ $@


############################################################
# MultiBoosterDataset Computation
############################################################

frame_classifier_full.mbd: 	$(addprefix frame_mbds/decimated/, $(addsuffix .mbd, $(NEGATIVES) $(TRAINING_DATA) $(HOLDOUT_DATA))) \
				$(addprefix frame_mbds/, $(addsuffix .mbd, $(ADDITIONAL_TRAINING_TMS)))
	mkdir -p $(@D)
	rosrun multibooster multitool --join $^ $@

frame_classifier_adbf.mbd: 	$(addprefix frame_mbds/decimated/, $(addsuffix .mbd, $(NEGATIVES) $(TRAINING_DATA))) \
				$(addprefix frame_mbds/, $(addsuffix .mbd, $(ADDITIONAL_TRAINING_TMS)))
	mkdir -p $(@D)
	rosrun multibooster multitool --join $^ $@

track_classifier.mbd: $(addprefix track_mbds/, $(addsuffix .mbd, $(NEGATIVES) $(TRAINING_DATA) $(ADDITIONAL_TRAINING_TMS)))
	mkdir -p $(@D)
	rosrun multibooster multitool --join $^ $@

frame_mbds/decimated/%.mbd: frame_mbds/%.mbd
	mkdir -p $(@D)
	rosrun multibooster multitool --decimateDataset 0.2 $^ $@

.SECONDARY: $(addprefix frame_mbds/, $(addsuffix .mbd, $(TRAINING_DATA) $(NEGATIVES) $(HOLDOUT_DATA))) # Don't delete the un-decimated datasets.

frame_mbds/%.mbd: labeled_tracks/%.tm
	mkdir -p $(@D)
	$(FRAME_DATASET_EXTRACTOR) $^ $@

frame_mbds/%.mbd: background_tracks/%.tm
	mkdir -p $(@D)
	$(FRAME_DATASET_EXTRACTOR) $^ $@

track_mbds/%.mbd: labeled_tracks/%.tm
	mkdir -p $(@D)
	$(TRACK_DATASET_EXTRACTOR) $^ $@

track_mbds/%.mbd: background_tracks/%.tm
	mkdir -p $(@D)
	$(TRACK_DATASET_EXTRACTOR) $^ $@


############################################################
# Automatic Track Extraction
############################################################

extract_tracks: 	$(addsuffix .tm, $(addprefix background_tracks/, $(NEGATIVES))) \
			$(addsuffix .tm, $(addprefix unlabeled_tracks/, $(LOGFILES)))

unlabeled_tracks/%.tm:
	mkdir -p $(@D)
	$(EXTRACT_TRACKS) logfiles/$(notdir $(basename $@)).log.gz $@

background_tracks/%.tm:
	mkdir -p $(@D)
	$(EXTRACT_TRACKS) negatives_logfiles/$(notdir $(basename $@)).log.gz $@
	track_tools --label-all background $@ $@ # Untested.  This was done by hand.
