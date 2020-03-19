
############################################################
# -- Global settings.
############################################################

SHELL			:= /bin/bash
RACE_ROOT		:= ../../..
export ROS_PACKAGE_PATH	:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

# These must exist.
SEQ_DIR := sequences

all:
	@echo You should probably be more specific.


############################################################
# -- Programs
############################################################

KINECT_CUT := rosrun dst kinect_cut 


############################################################
# -- Params
############################################################


############################################################
# -- Experiments
############################################################

results:
	mkdir -p $@
	for dir in `ls $(SEQ_DIR)/`; do \
	echo $$dir; \
	NUM_THREADS=40 DEBUG= $(KINECT_CUT) $(SEQ_DIR)/$$dir; \
	mv debug $@/$$dir; \
	done

visualize: results
	for dir in `ls results/`; do \
	echo $$dir; \
	rosrun dst segmentation_visualizer results/$$dir; \
	done