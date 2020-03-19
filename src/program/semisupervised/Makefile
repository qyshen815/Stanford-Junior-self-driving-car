include ../../Makefile.conf

MODULE_NAME = SEMISUPERVISED

IFLAGS += -Wno-unused-parameter -Wno-deprecated -Wno-deprecated-declarations
LFLAGS += -ltrack_tools $(GLOBAL_LIBS)

SOURCES = semisupervised.cpp data_quantity_experiment.cpp linear_solve.cpp test_eigen.cpp data_quantity_experiment2.cpp effort_experiment.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = 	semisupervised data_quantity_experiment linear_solve test_eigen data_quantity_experiment2 effort_experiment system_call  \
		generate_descriptor_graph synthetic_example projected_example

TEST_TARGETS = 
EVAL_TARGETS = 

ROS_DEPS = 	multibooster cluster_descriptors sensor_msgs unconstrained_optimization matplotlib_interface \
		eigen_extensions point_cloud_mapping bag_of_tricks serializable

# rules

semisupervised: semisupervised.o semisupervised_learner.o

data_quantity_experiment: data_quantity_experiment.o semisupervised_learner.o

data_quantity_experiment2: data_quantity_experiment2.o semisupervised_learner.o

effort_experiment: effort_experiment.o semisupervised_learner.o

linear_solve: linear_solve.o

test_eigen: test_eigen.o

system_call: system_call.o

generate_descriptor_graph: generate_descriptor_graph.o

synthetic_example: synthetic_example.o

projected_example: projected_example.o

include ../../Makefile.rules
