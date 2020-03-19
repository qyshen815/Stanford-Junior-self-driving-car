############################################################
# Global settings
############################################################

.SECONDEXPANSION: # Allows use of automatic vars in prerequisites.
.SECONDARY: # Prevents deletion of intermediate targets in a chain of implicit rules.
SHELL := /bin/bash
RACE_ROOT := /earth/driving/track_classification
export DESCRIPTORS:=no_rp
export NUM_THREADS:=40
export ROS_PACKAGE_PATH:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

############################################################
# Data
############################################################

FRAME_MULTIBOOSTER := frame_classifier_full.mb
TRACK_MULTIBOOSTER := track_classifier.mb
COMBINED_CLASSIFIER_WEIGHTS := weights.eig
INTRINSICS := t2i_superzoom_video_calib.yaml
LOG_DIR = logs

# Each logfile needs a log.gz, vlf, vlf.index.gz, avi.extrinsics.eig, avi.sync_offset.eig, avi.timestamp
# SUPERZOOM_LOGFILES := 	$(LOG_DIR)/2010-11-22_overhead/at_panama_parking_overhead2-11-22-2010_13-51-29 \
# 			$(LOG_DIR)/2010-11-22_overhead/at_panama_parking_overhead1-11-22-2010_13-41-50
#SUPERZOOM_LOGFILES := 	$(LOG_DIR)/2010-11-22_overhead/at_panama_parking_overhead2-11-22-2010_13-51-29
SUPERZOOM_LOGFILES := 	$(LOG_DIR)/2011-03-30_panama_overhead/panama-03-30-2011_11-55-21 \
			$(LOG_DIR)/2011-03-30_panama_overhead/panama-03-30-2011_12-11-15 \
			$(LOG_DIR)/2011-03-30_panama_overhead/panama-03-30-2011_12-38-57 \
			$(LOG_DIR)/2011-03-30_panama_overhead/panama-03-30-2011_12-24-00

FIFTY_MM_LOGFILES :=    $(LOG_DIR)/2011-04-07_hoover/overhead1-04-07-2011_12-55-40 \
			$(LOG_DIR)/2011-04-07_hoover/overhead2-04-07-2011_13-09-21 \
			$(LOG_DIR)/2011-04-07_hoover/overhead3-04-07-2011_13-23-01 \
			$(LOG_DIR)/2011-04-07_hoover/overhead4-04-07-2011_13-39-58 \
			$(LOG_DIR)/2011-04-07_hoover/overhead5-04-07-2011_13-53-33 \
			$(LOG_DIR)/2011-04-07_hoover/overhead6-04-07-2011_14-07-56 \
			$(LOG_DIR)/2011-04-07_hoover/overhead7-04-07-2011_14-21-30 \
			$(LOG_DIR)/2011-04-07_hoover/overhead8-04-07-2011_15-10-29 \
			$(LOG_DIR)/2011-04-07_hoover/overhead9-04-07-2011_15-24-04

FIFTY_MM_EXTRA_LOGFILES := 	$(LOG_DIR)/2011-04-07_hoover/serra_mall-04-07-2011_15-51-46


#$(LOG_DIR)/2011-03-30_panama_overhead/panama-03-30-2011_11-39-57 # This log has major synchronization drift.  Why?

#LOGFILES := $(SUPERZOOM_LOGFILES) $(FIFTY_MM_LOGFILES)
#LOGFILES := $(SUPERZOOM_LOGFILES)
LOGFILES := $(FIFTY_MM_LOGFILES)	

############################################################
# Programs
############################################################

PARALLEL := ~/parallel/parallel-20110125/src/parallel

VLF_INDEX := $(RACE_ROOT)/bin/vlf_index
DUMP_OBJECTS := rosrun image_labeler dump_objects
EXTRACT_TRACKS := RACE_ROOT=$(RACE_ROOT) python $(RACE_ROOT)/src/program/perception/extract_tracks_silently.py
TRACK_STATS := $(RACE_ROOT)/src/program/track_tools/track_tools --status
TRACK_PLOTS := $(RACE_ROOT)/src/program/track_tools/track_plots
TRACK_TOOLS := $(RACE_ROOT)/src/program/track_tools/track_tools
TRACK_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/track_classifier
EXTERNAL_TRACK_PROJECTOR := $(RACE_ROOT)/src/program/external_camera_projection/external_track_projector
CALIBRATE_EXTERNAL_CAMERA := $(RACE_ROOT)/src/program/external_camera_projection/calibrate_external_camera
PROJECT_INTO_EXTERNAL_CAMERA := $(RACE_ROOT)/src/program/external_camera_projection/project_into_external_camera

############################################################
# Everything
############################################################

all: 	$(addprefix videos/, $(addsuffix _external_projection.avi, $(LOGFILES))) \
	cropped_objects 


############################################################
# Track projection
############################################################

cropped_objects: $(addprefix datasets/, $(LOGFILES))
	$(DUMP_OBJECTS) $^ $@

datasets: $(addprefix datasets/, $(addsuffix /converted, $(LOGFILES)))

datasets/%/converted: datasets/%
	cd $(@D)/images && $(PARALLEL) convert {}.png {}.jpg ::: `ls *.png | awk -F. '{print $$1 "." $$2}'`
	rm $(@D)/images/*.png
	sed -i 's/png/jpg/g' $(@D)/annotations.txt
	touch $@

datasets/%: %.vlf.index.gz %.yaml %.avi %.avi.timestamp %.avi.sync_offset.eig %.avi.extrinsics.eig classified_tracks/%.tm $(FRAME_MULTIBOOSTER)
	mkdir -p $(@D)
	START_SEC=545 $(EXTERNAL_TRACK_PROJECTOR) $(FRAME_MULTIBOOSTER) $*.yaml $*.avi classified_tracks/$*.tm $@  # This gives us (at most) the last 3 minutes, 55 seconds of overhead{1-9}.

.PHONY: parallel_test
parallel_test:
	cd $@ && $(PARALLEL) convert {}.png {}.jpg ::: `ls *.png | awk -F. '{print $$1 "." $$2}'`


############################################################
# Track classification
############################################################

all_classified_tracks: $(addsuffix .tm, $(addprefix classified_tracks/, $(LOGFILES)))

classified_tracks/%.tm: $(FRAME_MULTIBOOSTER) $(TRACK_MULTIBOOSTER) $(COMBINED_CLASSIFIER_WEIGHTS) unlabeled_tracks/%.tm
	mkdir -p $(@D)
	$(TRACK_CLASSIFIER) $^ $@


############################################################
# Automatic track extraction
############################################################

all_unlabeled_tracks: $(addsuffix .tm, $(addprefix unlabeled_tracks/, $(LOGFILES)))

unlabeled_tracks/%.tm: %.log.gz
	mkdir -p $(@D)
	$(EXTRACT_TRACKS) $^ $@


############################################################
# External camera projection videos
############################################################

videos: $(addprefix videos/, $(addsuffix _external_projection.avi, $(LOGFILES)))

videos/%_external_projection.avi: %.yaml %.vlf.index.gz %.avi %.avi.timestamp %.avi.sync_offset.eig %.avi.extrinsics.eig
	mkdir -p videos/$*
	SCALE=0.5 $(PROJECT_INTO_EXTERNAL_CAMERA) $< $* videos/$*/images
	mencoder "mf://videos/$*/images/*.png" -mf fps=20.0 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=4800000 -vf scale=960:544 -o $@
	rm -rf videos/$*

all_depth_data: $(addprefix depth_data/, $(SUPERZOOM_LOGFILES))

depth_data/%: %.yaml %.vlf.index.gz %.avi %.avi.timestamp %.avi.sync_offset.eig %.avi.extrinsics.eig
	mkdir -p $(@D)
	SAVE_DEPTH= $(PROJECT_INTO_EXTERNAL_CAMERA) $< $* $@


############################################################
# Log Preparation
############################################################

log_prep: 	$(addsuffix .vlf.index.gz, $(LOGFILES)) \
		$(addsuffix .avi, $(LOGFILES)) \
		$(addsuffix .avi.timestamp, $(LOGFILES)) \
		$(addsuffix .yaml, $(LOGFILES))

calibrate_all: $(addsuffix .avi.extrinsics.eig, $(LOGFILES))

%.vlf.index.gz: %.log.gz %.vlf
	$(VLF_INDEX) $^

%.avi.timestamp: %.MOV
	exiftool $^ | grep '^Track Create Date' | awk '{print $$5, $$6}' | sed 's/:/-/' | sed 's/:/-/' > $@

# 29.97 is the right value - 30.0 is wrong, despite what exiftool says.
%.avi: %.MOV
	mencoder $^ -noskip -mc 0 -vf harddup -ofps 29.97 -nosound -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=24000000 -o $@

%.avi.sync_offset.eig: %.yaml %.avi.timestamp %.avi %.vlf.index.gz
	@echo You need to do the extrinsic calibration for this log.
	@echo Use the following:
	@echo $(CALIBRATE_EXTERNAL_CAMERA) $< $*
	$(CALIBRATE_EXTERNAL_CAMERA) $< $*
#terminate_makefile

%.avi.extrinsics.eig: %.yaml %.avi.timestamp %.avi %.vlf.index.gz
	@echo You need to do the extrinsic calibration for this log.
	@echo Use the following:
	@echo $(CALIBRATE_EXTERNAL_CAMERA) $< $*
	$(CALIBRATE_EXTERNAL_CAMERA) $< $*
#terminate_makefile


define SUPERZOOM_INTRINSICS_RULE_GENERATOR
$(1).yaml: t2i_superzoom_video_calib.yaml
	cp $$^ $$@
endef
$(foreach VAL,$(SUPERZOOM_LOGFILES),$(eval $(call SUPERZOOM_INTRINSICS_RULE_GENERATOR,$(VAL))))

define FIFTY_MM_INTRINSICS_RULE_GENERATOR
$(1).yaml: t2i_50mm_video_calib.yaml
	cp $$^ $$@
endef
$(foreach VAL,$(FIFTY_MM_LOGFILES) $(FIFTY_MM_EXTRA_LOGFILES),$(eval $(call FIFTY_MM_INTRINSICS_RULE_GENERATOR,$(VAL))))


############################################################
# Special cases
############################################################

unlabeled_tracks/logs/2011-03-30_panama_overhead/panama-03-30-2011_12-24-00.tm: unlabeled_tracks/logs/2011-03-30_panama_overhead/panama-03-30-2011_12-11-15.tm
	cd $(@D) && ln -s $(^F) $(@F)

classified_tracks/logs/2011-03-30_panama_overhead/panama-03-30-2011_12-24-00.tm: classified_tracks/logs/2011-03-30_panama_overhead/panama-03-30-2011_12-11-15.tm
	cd $(@D) && ln -s $(^F) $(@F)

