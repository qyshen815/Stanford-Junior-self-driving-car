total:		     rospkgs home export make-alldirs

include Makefile.conf

EXT_PACKAGES = 	     ipc glui ANN #rote

INTERFACE_PACKAGES = heartbeat ipc param_server can applanix \
		     playback estop passat controller planner error simulator \
		     localize perception pidcontrol gls radar hci \
		     power velodyne timesync healthmon latency ghostcar \
         ladybug camera trajectory_points traffic_lights event_notify \
         aw_vehicle aw_navigator aw_chsm_planner\
         trajectory_points ldlrs laser

LIBRARY_PACKAGES =

ifeq ($(ARCH), Darwin)
LIBRARY_PACKAGES += librt_stub
endif

LIBRARY_PACKAGES += global sockutils serial logio lltransform kdtree aio \
		     gl_support image ned pointlist project_data \
		     pswrap textures blf vehiclemodels passatmodel \
		     trajectory videoout rndf imagery b64 project joystick \
		     grid usbfind fsm shm velocore sla gpp gnuplot dc1394camera \
		     gtk_support playback filter rndf_search \
                     aw_core aw_timestamp aw_geometry fastjpeg ladybug \
	 	     curve_smoothing aw_rndf aw_mdf aw_chsm_planning \
                     aw_topology poly_traj  car_list graph_cut \
		     track_descriptors track_tools

PROGRAM_PACKAGES =   can applanix param_server param_edit \
                     logger playback playback_control util vehiclesim controller \
                     trajectory_utils passat pidcontrol watchdog \
                     fake_planner log_utils progress \
                     error estop power camera ladybug ladybug_calibrator \
                     velodyne perception_view ipc_utils \
                     timesync hci healthmon \
                     ghostcar vlf_view slam blf_utils camera_view \
                     localize_velodyne event_notify aw_planner power_utils \
		     track_tools perception semisupervised external_camera_projection \
                     online_learning

ROS_PACKAGES = 	ANN cgal chamfer_matching cluster_descriptors \
		FLANN google-perftools point_cloud_mapping \
		multibooster pipeline qpOASES libdevil \
		cminpack linear_kalman_filter kalman_multitracker matplotlib_interface \
		name_mapping performance_statistics unconstrained_optimization \
		image_descriptor_nodes eigen_extensions image_labeler \
		bag_of_tricks epnp online_learning name_mapping2


ifeq (1,$(HAVE_IPP))
EXT_PACKAGES +=      libjpeg.ipp
endif

#TEST_PACKAGES =      param_server rndf system_test #velodyne multicentral

include Makefile.toplevelrules
