############################################################
# Global settings
############################################################

.SECONDEXPANSION: # Allows use of automatic vars in prerequisites.
.SECONDARY: # Prevents deletion of intermediate targets in a chain of implicit rules.

export DESCRIPTORS:=no_rp
export NUM_THREADS:=40
export ROS_PACKAGE_PATH:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg


############################################################
# Programs
############################################################

EXTRACT_TRACKS := RACE_ROOT=$(RACE_ROOT) python $(RACE_ROOT)/src/program/perception/extract_tracks_silently.py
TRACK_STATS := $(RACE_ROOT)/src/program/track_tools/track_tools --status
TRACK_PLOTS := $(RACE_ROOT)/src/program/track_tools/track_plots
TRACK_TOOLS := $(RACE_ROOT)/src/program/track_tools/track_tools
TRACK_CLASSIFIER := $(RACE_ROOT)/src/program/track_tools/track_classifier
TRACK_PROJECTOR_DETAILED := DRAW_HULL= DRAW_PTS= DRAW_BACKGROUND= $(RACE_ROOT)/src/program/track_tools/track_projector
TRACK_PROJECTOR_SIMPLE := DRAW_INTERPOLATED_BBOX= $(RACE_ROOT)/src/program/track_tools/track_projector
TRACK_PROJECTION_DATASET_GENERATOR := $(RACE_ROOT)/src/program/track_tools/track_projection_dataset_generator
BLF_INDEX := $(RACE_ROOT)/bin/blf-index
VLF_INDEX := $(RACE_ROOT)/bin/vlf_index


############################################################
# Data
############################################################

FRAME_MULTIBOOSTER := frame_classifier.mb
TRACK_MULTIBOOSTER := track_classifier.mb
COMBINED_CLASSIFIER_WEIGHTS := weights.eig
LOG_DIR := logs

# Logs with old calibration
#LOGFILES := 	2010-09-17/synchronization_test-fast_cars-09-17-2010_11-22-24 \
#		2010-10-06_dataset_collection_around_campus/driving_to_hospital_crosswalk-10-06-2010_16-20-17 \
#		2010-10-06_dataset_collection_around_campus/driving_to_intersection-10-06-2010_15-46-12 \
#		2010-10-06_dataset_collection_around_campus/driving_to_parking_lot-10-06-2010_16-04-15 \
#		2010-10-06_dataset_collection_around_campus/gates_parking_lot_view_of_crosswalk-10-06-2010_16-16-51 \
#		2010-10-06_dataset_collection_around_campus/hospital_crosswalk_to_oval-10-06-2010_16-22-50 \
#		2010-10-06_dataset_collection_around_campus/oval-10-06-2010_16-29-00 \
#		2010-10-06_dataset_collection_around_campus/oval_to_vail-10-06-2010_16-34-06 \
#		2010-10-06_dataset_collection_around_campus/parking_lot_plus-10-06-2010_16-08-30 \
#		2010-10-06_dataset_collection_around_campus/sitting_at_escondido_and_campus_loop-10-06-2010_15-52-06 \
		2010-10-06_dataset_collection_around_campus/ungrad_area-10-06-2010_16-12-40

# Logs with new calibration
 LOGFILES := 	2010-10-31_mostly_cars/marguerites-10-31-2010_17-13-53 \
		2010-10-31_mostly_cars/run1-10-31-2010_17-09-21 \
		2010-10-31_mostly_cars/run2-10-31-2010_17-18-52 \
		2010-10-31_mostly_cars/run3-10-31-2010_17-26-53 \
		2010-10-31_mostly_cars/run4-10-31-2010_17-34-01 \
		2010-10-31_mostly_cars/run5-10-31-2010_17-39-25 \
		2010-10-31_mostly_cars/run6-10-31-2010_17-52-08 \
		2010-10-31_mostly_cars/run7-10-31-2010_17-58-23





############################################################
# Track projection
############################################################

all: all_datasets all_videos

all_datasets: 	all_indices \
		$(addsuffix _cam2, $(addprefix datasets/, $(LOGFILES))) \
		$(addsuffix _cam1, $(addprefix datasets/, $(LOGFILES)))

datasets/%_cam2: classified_tracks/%.tm $(LOG_DIR)/%.llf
	mkdir -p $(@D) # Create all directories except the target directory.  $(@D) doesn't include the target dir.
	$(TRACK_PROJECTION_DATASET_GENERATOR) $^ 2 $@
	cd $@/images; for file in `ls *.png`; do echo $$file; convert $$file $${file%.png}.jpg; rm $$file; done
	sed -i 's/png/jpg/g' $@/annotations.txt

datasets/%_cam1: classified_tracks/%.tm $(LOG_DIR)/%.llf
	mkdir -p $(@D) # Create all directories except the target directory.  $(@D) doesn't include the target dir.
	$(TRACK_PROJECTION_DATASET_GENERATOR) $^ 1 $@
	cd $@/images; for file in `ls *.png`; do echo $$file; convert $$file $${file%.png}.jpg; rm $$file; done
	sed -i 's/png/jpg/g' $@/annotations.txt

all_videos: 	$(addsuffix -detailed.avi, $(addprefix videos/, $(LOGFILES))) \
		$(addsuffix -simple.avi, $(addprefix videos/, $(LOGFILES)))

videos/%-detailed.avi: classified_tracks/%.tm $(LOG_DIR)/%.llf
	mkdir -p $(@D)
	$(TRACK_PROJECTOR_DETAILED) $^ $@
	mencoder $@ -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=1800 -vf eq2=1.0:1.9:0.4:1.0 -o $@.2
	mv $@.2 $@

videos/%-simple.avi: classified_tracks/%.tm $(LOG_DIR)/%.llf
	mkdir -p $(@D)
	$(TRACK_PROJECTOR_SIMPLE) $^ $@
	mencoder $@ -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=1800 -vf eq2=1.0:1.9:0.4:1.0 -o $@.2
	mv $@.2 $@


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

unlabeled_tracks/%.tm: $(LOG_DIR)/%.log.gz
	mkdir -p $(@D)
	$(EXTRACT_TRACKS) $^ $@


############################################################
# Log Preparation
############################################################

all_indices: 	$(addprefix $(LOG_DIR)/, $(addsuffix .llf.idx.gz, $(LOGFILES))) \
		$(addprefix $(LOG_DIR)/, $(addsuffix .vlf.index.gz, $(LOGFILES)))

$(LOG_DIR)/%.llf.idx.gz: $(LOG_DIR)/%.llf
	$(BLF_INDEX) $^

$(LOG_DIR)/%.vlf.index.gz: $(LOG_DIR)/%.log.gz $(LOG_DIR)/%.vlf
	$(VLF_INDEX) $^
