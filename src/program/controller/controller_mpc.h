#ifndef CONTROLLER_MPC_H
#define CONTROLLER_MPC_H

#include <Eigen/Core>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <can_interface.h>
#include <passat_constants.h>
#include <planner_messages.h>
#include <can_messages.h>
#include <global.h>
#include <poly_traj.h>
#include <vehicle.h>


#define NUM_STATES 6
#define NUM_CONTROLS 2

namespace vlr {

class MPCController {
 public:
  MPCController();
  ~MPCController() {};

  void run() { m_ipc->Dispatch(); }
  void controlLoop();
  void changeParams();

 private:
  void getState();
  void getDesiredStates();
  void getThrottleTorque();

  void mpcLQR(const Eigen::Matrix<double, NUM_STATES, 1> &s0,
	      const Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> &u0,
	      const Eigen::Matrix<double, NUM_STATES, Eigen::Dynamic> &s_star,
	      const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev,
	      Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> *u_out);

  void laneFollow(const Eigen::Matrix<double, NUM_STATES, 1> &s0,
		  const Eigen::Matrix<double, NUM_STATES, 1> &s_star,
		  Eigen::Matrix<double, NUM_CONTROLS, 1> *u_out);

  void getThrottleTorqueOriginal();
		  
  
  void dynamics(const Eigen::Matrix<double, NUM_STATES, 1> &s,
		const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
		Eigen::Matrix<double, NUM_STATES, 1> *s_dot,
		Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0,
		Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

  void simulateEuler(const Eigen::Matrix<double, NUM_STATES, 1> &s,
		     const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
		     Eigen::Matrix<double, NUM_STATES, 1> *s_next,
		     Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0,
		     Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);
  
  void simulateRK4(const Eigen::Matrix<double, NUM_STATES, 1> &s,
		   const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
		   Eigen::Matrix<double, NUM_STATES, 1> *s_next,
		   Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0,
		   Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);


  double simulatorThrottle(double u, double v, double th_dot, double u_d,
			   double del);

  dgc::IpcInterface *m_ipc;
  dgc::ApplanixPose m_applanix;
  dgc::CanStatus m_can;
  TrajectoryPoints2D m_traj;

  Eigen::Matrix<double, NUM_STATES,1> m_state;
  Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> m_controls;
  Eigen::Matrix<double, NUM_STATES, Eigen::Dynamic> m_des_states;
  Eigen::Vector2d m_errors;
  
  Eigen::Matrix<double, NUM_CONTROLS, 1> m_throttle_torque;
  Eigen::Matrix<double, NUM_CONTROLS, 1> m_ctl_err;
  Eigen::Matrix<double, NUM_CONTROLS, 1> m_ctl_err_vel;
  Eigen::Matrix<double, NUM_CONTROLS, 1> m_ctl_err_int;
  

  //MSKenv_t m_mosek_env;

  double m_vel_err_int;
  double m_last_dtheta;
  

  // parameters
  vehicle_state p_vs;
  bool p_torque_mode;
  int p_horizon;
  double p_hertz;
  
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> p_Q;
  Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R;
  Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R_delta;

  double p_q_lon, p_q_lat, p_q_theta, p_q_u, p_q_v, p_q_theta_dot;
  double p_r_udot, p_r_delta, p_rd_udot, p_rd_delta;

  double p_vel_smooth, p_int_decay;
  double p_k_throttle, p_d_throttle, p_i_throttle, p_ff_throttle;
  double p_k_torque, p_d_torque, p_i_torque, p_ff_torque;
  double p_throttle_smooth, p_torque_smooth;

  double p_p_cte, p_d_cte, p_k_yawrate, p_k_aggressive;
  double p_p_lon_err;

  double p_max_vel_int, p_k_cruise_int, p_k_accel, p_k_decel;
  double p_kp_torque, p_kd_torque;
};

} // namespace vlr
#endif  
