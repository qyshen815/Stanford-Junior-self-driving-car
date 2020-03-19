RACE_ROOT := /earth/driving/track_classification
export NUM_THREADS:=80
export ROS_PACKAGE_PATH:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg
.SECONDEXPANSION: # Allows use of automatic vars in prerequisites.

DATA_DIR := hands/dataset5
# Positives, then negatives.  No multiclass yet.
TRAINING_DIRS := $(DATA_DIR)/training/stophand $(DATA_DIR)/training/background
# Positives, then negatives.  No multiclass yet.
TESTING_DIRS := $(DATA_DIR)/testing/stophand $(DATA_DIR)/testing/background

TRAIN_FRAME_CLASSIFIER := NCANDIDATES=10 MAX_WCS=1000 rosrun multibooster multitool --train
EVALUATE_HAND_CLASSIFIER := rosrun hand_classifier evaluate
EXTRACT_MBD := rosrun hand_classifier extract_mbd
AUGMENT_DATASET := bash augment_dataset-rotations_only.sh


############################################################
# Testing
############################################################

$(DATA_DIR)/results.txt: $(DATA_DIR)/hands.mb
	$(EVALUATE_HAND_CLASSIFIER) $^ $(TESTING_DIRS) $(DATA_DIR)/results


############################################################
# Training
############################################################

$(DATA_DIR)/hands.mb: $(DATA_DIR)/hands.mbd
	$(TRAIN_FRAME_CLASSIFIER) $^ $@

$(DATA_DIR)/hands.mbd: $(DATA_DIR)/augmented
	$(EXTRACT_MBD) $(TRAINING_DIRS) $@


############################################################
# Training set augmentation
############################################################

$(DATA_DIR)/augmented:
	$(AUGMENT_DATASET) $(DATA_DIR)/training/stophand
	touch $@