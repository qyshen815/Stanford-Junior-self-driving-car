include ../../Makefile.conf

CXXFLAGS += -g
IFLAGS += -Wno-unused-parameter

#Cruft flags from perception.
LFLAGS += -law_rndf -law_geometry -lipc_interface -lfilter
LFLAGS += `gsl-config --libs` `pkg-config ImageMagick --libs`
LFLAGS += -lcar_list -lvelo_support -llogio -lsockutils
LFLAGS += -lgrid
LFLAGS += $(GRAPHICS_LIBS)
LFLAGS += -llltransform -lsockutils -lCGAL -law_rndf -lapplanix_interface
LFLAGS += -llocalize_interface -lIL -lftgl #-lippcore
IFLAGS += `pkg-config --cflags libavformat libavutil libswscale freetype2` #This doesn't work on 9.10.
LFLAGS += `pkg-config --libs libavcodec libavformat libavutil libswscale freetype2`
IFLAGS += -I/usr/include/libavformat -I/usr/include/libavcodec -I/usr/include/libswscale #For 9.10.
IFLAGS += -I/usr/include/freetype2/
LFLAGS += -ltrack_descriptors -ltrack_tools $(INTERFACE_LIBS) $(GLOBAL_LIBS) 


ifeq (1,$(HAVE_IPP))
LFLAGS += -ltrack_tools -lfastjpeg -lladybug_model -lladybug_video -lladybug_frameparser -limage -lblf -lasync_writer -lglobal -lfastjpeg -lladybug_interface $(IPPLIBS) -ldc1394
endif



MODULE_NAME = TRACK_TOOLS

SOURCES = check_mbd_duplicates.cpp metaclassifier_dataset_extractor.cpp dbf_experiments.cpp print_matrix.cpp test_metaclassifier.cpp track_classifier.cpp track_classifier_frame_only.cpp evaluate_frame_classifier.cpp evaluate_track_classifier.cpp evaluate_combined_classifier.cpp train_combined_classifier.cpp frame_dataset_extractor.cpp track_dataset_extractor.cpp track_projector.cpp track_projection_dataset_generator.cpp track_stats.cpp track_plots.cpp frame_extractor.cpp track_tools.cpp track_visualizer.cpp train_metaclassifier.cpp track_filterer.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = 

GRAPHICS_TARGETS = track_visualizer 
TARGETS = 	evaluate_frame_classifier evaluate_track_classifier evaluate_combined_classifier train_combined_classifier\
		track_stats check_mbd_duplicates track_classifier track_classifier_frame_only \
		frame_dataset_extractor track_dataset_extractor track_tools metaclassifier_dataset_extractor \
		train_metaclassifier test_metaclassifier \
		print_matrix track_filterer track_plots frame_extractor dump_pipeline_graphviz \
		track_projection_dataset_generator

ifeq (1,$(HAVE_IPP))
TARGETS += track_projector
endif


#create_contrived_test_set

TEST_TARGETS = 
EVAL_TARGETS = 

ROS_DEPS = multibooster cgal google-perftools cluster_descriptors pipeline sensor_msgs opencv2 unconstrained_optimization matplotlib_interface eigen_extensions image_labeler point_cloud_mapping

# rules

track_projection_dataset_generator: track_projection_dataset_generator.o

dump_pipeline_graphviz: dump_pipeline_graphviz.o

track_filterer: track_filterer.o

#create_contrived_test_set: create_contrived_test_set.o

print_matrix: print_matrix.o

test_metaclassifier: test_metaclassifier.o dbf_experiments.o

train_metaclassifier: train_metaclassifier.o dbf_experiments.o

metaclassifier_dataset_extractor: metaclassifier_dataset_extractor.o

track_tools: track_tools.o

track_plots: track_plots.o

frame_extractor: frame_extractor.o

track_projector: track_projector.o

track_stats: track_stats.o

track_classifier: track_classifier.o

track_classifier_frame_only: track_classifier_frame_only.o

evaluate_frame_classifier: evaluate_frame_classifier.o

evaluate_track_classifier: evaluate_track_classifier.o

evaluate_combined_classifier: evaluate_combined_classifier.o

train_combined_classifier: train_combined_classifier.o

track_visualizer: track_visualizer.o

frame_dataset_extractor: frame_dataset_extractor.o

track_dataset_extractor: track_dataset_extractor.o

check_mbd_duplicates: check_mbd_duplicates.o

include ../../Makefile.rules
