#include <planner_interface.h>
#include <can_interface.h>
#include <passat_interface.h>
#include <heartbeat_interface.h>
#include <passat_constants.h>
#include <planner_messages.h>
#include <can_messages.h>
#include <controller_messages.h>
#include <global.h>
#include <poly_traj.h>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <fstream>

#include "controller_mpc.h"

using namespace std;
using namespace dgc;
using namespace Eigen;

ofstream f_data("data.dat");

vlr::MPCController mpc;
void control_callback(vlr::MPCController *controller) {controller->controlLoop();}
void param_callback() { mpc.changeParams(); }

int main()
{
  mpc.run();
}

namespace vlr {

// Contstructor, initialize ipc, params, etc
MPCController::MPCController()
{
  // connect to IPC
  m_ipc = new IpcStandardInterface();
  if (m_ipc->ConnectLocked("lqr_controller") < 0)
    dgc_fatal_error("Could not connect to IPC network");

  int argc = 1;
  char *argv[] = {"controller_mpc"};

  // read parameters
  ParamInterface *pint = new ParamInterface(m_ipc);
  Param params[] = {
    {"lqrcontroller", "hz", DGC_PARAM_DOUBLE, &p_hertz, 0, NULL},
    {"lqrcontroller", "horizon", DGC_PARAM_INT, &p_horizon, 0, NULL},
    {"lqrcontroller", "torque_mode", DGC_PARAM_ONOFF, &p_torque_mode,0,NULL},
    {"lqrcontroller", "cost_lon", DGC_PARAM_DOUBLE, &p_q_lon, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_lat", DGC_PARAM_DOUBLE, &p_q_lat, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_theta", DGC_PARAM_DOUBLE, &p_q_theta, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_vx", DGC_PARAM_DOUBLE, &p_q_u, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_vy", DGC_PARAM_DOUBLE, &p_q_v, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_theta_dot", DGC_PARAM_DOUBLE, &p_q_theta_dot, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_accel", DGC_PARAM_DOUBLE, &p_r_udot, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_steer", DGC_PARAM_DOUBLE, &p_r_delta, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_delta_accel", DGC_PARAM_DOUBLE, &p_rd_udot, 1,
     ParamCB(param_callback)},
    {"lqrcontroller", "cost_delta_steer", DGC_PARAM_DOUBLE, &p_rd_delta, 1,
     ParamCB(param_callback)},

    {"lqrcontroller", "vel_smooth", DGC_PARAM_DOUBLE, &p_vel_smooth, 0, NULL},
    {"lqrcontroller", "int_decay", DGC_PARAM_DOUBLE, &p_int_decay, 0, NULL},
    {"lqrcontroller", "k_throttle", DGC_PARAM_DOUBLE, &p_k_throttle, 0, NULL},
    {"lqrcontroller", "d_throttle", DGC_PARAM_DOUBLE, &p_d_throttle, 0, NULL},
    {"lqrcontroller", "i_throttle", DGC_PARAM_DOUBLE, &p_i_throttle, 0, NULL},
    {"lqrcontroller", "ff_throttle", DGC_PARAM_DOUBLE, &p_ff_throttle, 0, NULL},
    {"lqrcontroller", "k_torque", DGC_PARAM_DOUBLE, &p_k_torque, 0, NULL},
    {"lqrcontroller", "d_torque", DGC_PARAM_DOUBLE, &p_d_torque, 0, NULL},
    {"lqrcontroller", "i_torque", DGC_PARAM_DOUBLE, &p_i_torque, 0, NULL},
    {"lqrcontroller", "ff_torque", DGC_PARAM_DOUBLE, &p_ff_torque, 0, NULL},
    {"lqrcontroller", "throttle_smooth", DGC_PARAM_DOUBLE, &p_throttle_smooth,
     0, NULL},
    {"lqrcontroller", "torque_smooth", DGC_PARAM_DOUBLE, &p_torque_smooth,
     0, NULL},

    {"controller", "p_cte", DGC_PARAM_DOUBLE, &p_p_cte, 0, NULL},
    {"controller", "d_cte", DGC_PARAM_DOUBLE, &p_d_cte, 0, NULL},
    {"controller", "k_yawrate", DGC_PARAM_DOUBLE, &p_k_yawrate, 0, NULL},
    {"controller", "k_aggressive", DGC_PARAM_DOUBLE, &p_k_aggressive, 0, NULL},

    {"controller", "vel_int_cap", DGC_PARAM_DOUBLE, &p_max_vel_int, 0, NULL},
    {"controller", "k_cruise_int", DGC_PARAM_DOUBLE, &p_k_cruise_int, 0, NULL},
    {"controller", "k_accel", DGC_PARAM_DOUBLE, &p_k_accel, 0, NULL},
    {"controller", "k_decel", DGC_PARAM_DOUBLE, &p_k_decel, 0, NULL},
    {"controller", "kp_torque", DGC_PARAM_DOUBLE, &p_kp_torque, 0, NULL},
    {"controller", "kd_torque", DGC_PARAM_DOUBLE, &p_kd_torque, 0, NULL}
    
  };
  
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
  p_Q.setZero();
  p_R.setZero();
  p_R_delta.setZero();
  p_vs.set_passat_params();
  changeParams();
  m_des_states.resize(NUM_STATES, p_horizon+1);
  m_controls.resize(NUM_CONTROLS, p_horizon);
  m_controls.setZero();
  m_errors.setZero();
  delete pint;

  m_ctl_err.setZero();
  m_ctl_err_vel.setZero();
  m_ctl_err_int.setZero();
  m_vel_err_int = 0;
  m_last_dtheta = 0;
  

  // initialize mosek
  //MSK_makeenv(&m_mosek_env, NULL, NULL, NULL, NULL);
  //MSK_linkfunctoenvstream(m_mosek_env, MSK_STREAM_LOG, NULL, printstr);
  //MSK_initenv(m_mosek_env);

  // subscribe to messages and set up timer
  m_applanix.timestamp = m_can.timestamp = m_traj.num_points = -1;
  m_ipc->Subscribe(ApplanixPoseID, &m_applanix); 
  m_ipc->Subscribe(CanStatusID, &m_can);
  m_ipc->Subscribe(TrajectoryPoints2DID, &m_traj);
  m_ipc->DefineMessage(ControllerTargetID);
  m_ipc->AddTimer( 1.0 / p_hertz, control_callback, this);
}

// parameter change callback
void MPCController::changeParams()
{
  p_Q(0,0) = p_q_lon;
  p_Q(1,1) = p_q_lat;
  p_Q(2,2) = p_q_theta;
  p_Q(3,3) = p_q_u;
  p_Q(4,4) = p_q_v;
  p_Q(5,5) = p_q_theta_dot;

  p_R(0,0) = p_r_udot;
  p_R(1,1) = p_r_delta;
  p_R_delta(0,0) = p_rd_udot;
  p_R_delta(1,1) = p_rd_delta;
}

// main controller loop
void MPCController::controlLoop()
{
  printf("applanix timestamp: %lf\ncan timestamp: %lf\n # trajectory points: %u\n", m_applanix.timestamp, m_can.timestamp, m_traj.num_points);
  if (m_applanix.timestamp <= -1 || m_can.timestamp <= -1 ||
      m_traj.num_points <= -1) return;

  // if we haven't gotten anything for a while, reset
  if (m_traj.points[0].t < m_applanix.timestamp - 10.0) {
    m_controls.setZero();
    m_throttle_torque.setZero();
    m_ctl_err.setZero();
    m_ctl_err_vel.setZero();
    m_ctl_err_int.setZero();
    m_vel_err_int = 0;
    m_last_dtheta = 0;
    cout << "No commands in a while, resetting controller..." << endl;
    return;
  }

  // compute desired control (delta, velocity)
  getState();
  getDesiredStates();
  //cout << m_errors.transpose() << endl;

  // shift controls and run MPC
  Matrix<double, NUM_CONTROLS, 1> u_prev = m_controls.col(0);
  Matrix<double, NUM_STATES, Dynamic> s_pred(NUM_STATES, p_horizon+1);
  Matrix<double, NUM_STATES, 1> s;
  
  m_controls.block(0,0,NUM_CONTROLS, p_horizon-1) =
    m_controls.block(0,1,NUM_CONTROLS, p_horizon-1);

  for (int i = 0; i < 1; i++) {
    mpcLQR(m_state, m_controls, m_des_states, u_prev, &m_controls);
  }
  //laneFollow(m_state, m_des_states.col(0), &u_prev);
  //m_controls.col(0) = u_prev;
  f_data << m_des_states(0,0) << " " << m_des_states(0,1) << " " << m_des_states(0,2) << " "
         << m_state(0) << " " << m_state(1) << " " << m_state(2) << endl;

  if (!p_torque_mode) {
    double throttle = simulatorThrottle(m_state(3,0), m_state(4,0),m_state(5,0),
					m_controls(0,0), m_controls(1,0));
    double steering_angle = m_controls(1,0) * p_vs.param.steering_ratio;

    cout << throttle << " " << steering_angle << endl;

    PassatActuatorAngleCommand(m_ipc, DGC_PASSAT_DIRECTION_FORWARD,
			       steering_angle, 30.0 * max(0.0, -throttle),
    			       max(0.0, throttle));
  } else {
    getThrottleTorque();
    cout << "steering: " << m_controls(1,0) << " " << m_throttle_torque(1) <<endl;
    
    PassatActuatorTorqueCommand(m_ipc, DGC_PASSAT_DIRECTION_FORWARD,
				m_throttle_torque(1),
				30.0 * max(0.0, -m_throttle_torque(0)),
				max(0.0, m_throttle_torque(0)));
  }

  // publish errors
  ControllerTarget target;
  strncpy(target.host, dgc_hostname(), 10);
  target.target_velocity = m_des_states(3,0);
  target.target_steering_angle = m_controls(1,0) * p_vs.param.steering_ratio;
  target.cross_track_error =
    -sin(m_des_states(2,0))*(m_state(0) - m_des_states(0,0)) +
    cos(m_des_states(2,0))*(m_state(1) - m_des_states(1,0));
  target.heading_error = (m_state(2) - m_des_states(2,0));
  target.timestamp = dgc_get_time();
  m_ipc->Publish(ControllerTargetID, &target);
}

// get current state of the car (really a delta state from desired state)
void MPCController::getState()
{
  double imu_to_cg = DGC_PASSAT_IMU_TO_FA_DIST; //p_vs.param.imu_to_cg_dist;
  m_state(0) = m_applanix.smooth_x + cos(m_applanix.yaw) * imu_to_cg;
  m_state(1) = m_applanix.smooth_y + sin(m_applanix.yaw) * imu_to_cg;
  
  m_state(2) = m_applanix.yaw;
  m_state(5) = m_applanix.ar_yaw;

  double cos_th = cos(m_state(2)), sin_th = sin(m_state(2));
  double v_e = m_applanix.v_east - imu_to_cg * m_state(5) * sin(m_state(2));
  double v_n = m_applanix.v_north + imu_to_cg * m_state(5) * cos(m_state(2));
  
  m_state(3) = cos_th * v_e + sin_th * v_n;
  m_state(4) = -sin_th * v_e + cos_th * v_n;
}


// get desired states from trajecotry and controls using Newton's method
void MPCController::getDesiredStates()
{
  double t = m_applanix.timestamp, alpha;
  double ra_to_cg = DGC_PASSAT_WHEEL_BASE;
  int j = 0;

  // find all desired positions
  TrajectoryPoint2D *p1, *p2;
  double dt = 1.0/p_hertz;
  for (int i = 0; i < p_horizon+1; i++) {
    while (m_traj.points[j+1].t < t && j < m_traj.num_points-2) j++;
    p1 = &m_traj.points[j];
    p2 = &m_traj.points[j+1];
    while (p2->theta - p1->theta > M_PI) p2->theta -= 2*M_PI;
    while (p2->theta - p1->theta < -M_PI) p2->theta += 2*M_PI; 

    // integrate to create a smoothed trajectory
    alpha = (t- p1->t) / (p2->t - p1->t);
    if (alpha > 1) alpha = 1;
    if (alpha < 0) alpha = 0;

    m_des_states(0,i) = (1-alpha)*p1->x + alpha*p2->x;
    m_des_states(1,i) = (1-alpha)*p1->y + alpha*p2->y;
    m_des_states(2,i) = (1-alpha)*p1->theta + alpha*p2->theta;
    m_des_states(0,i) += cos(m_des_states(2,i)) * ra_to_cg;
    m_des_states(1,i) += sin(m_des_states(2,i)) * ra_to_cg;
    m_des_states(3,i) = (1-alpha)*p1->v + alpha*p2->v;
    m_des_states(4,i) = 0.0;
    m_des_states(5,i) = m_des_states(3,i)*((1-alpha)*p1->kappa+alpha*p2->kappa);
    t += dt;
  }

  // normalize desired angles properly
  while (m_des_states(2,0) - m_state(2) > M_PI) m_des_states(2,0) -= 2*M_PI;
  while (m_des_states(2,0) - m_state(2) < -M_PI) m_des_states(2,0) += 2*M_PI;
  for (int i = 1; i < p_horizon+1; i++) {
    while (m_des_states(2,i) - m_des_states(2,i-1) > M_PI)
      m_des_states(2,i) -= 2*M_PI;
    while (m_des_states(2,i) - m_des_states(2,i-1) < -M_PI)
      m_des_states(2,i) += 2*M_PI;
  }


  m_errors += 0.05*(Rotation2D<double>(-m_state(2)).toRotationMatrix()*
		    (m_state.block(0,0,2,1)-m_des_states.block(0,0,2,1))
		    - m_errors);
}


void MPCController::getThrottleTorque()
{
  // integrate forward to see where velocity should be
  Matrix<double, NUM_STATES, 1> s;
  Matrix<double, NUM_CONTROLS, 1> err;

  simulateEuler(m_state, m_controls.col(0), &s);

  // compute errors, error derivatives (finite differences), a integrators
  err(0) = m_state(3) - s(3);
  err(1) = (M_PI/180)*m_can.steering_angle/DGC_PASSAT_STEERING_RATIO -
    m_controls(1,0);
  m_ctl_err_vel = p_vel_smooth*m_ctl_err_vel +
    (1-p_vel_smooth)*(err - m_ctl_err)*p_hertz;
  m_ctl_err_int = m_ctl_err_int * p_int_decay + err;
  m_ctl_err = err;

  // apply control
  m_throttle_torque(0) = p_throttle_smooth * m_throttle_torque(0) +
    (1 - p_throttle_smooth) * 
    (p_k_throttle * m_ctl_err(0) + p_d_throttle * m_ctl_err_vel(0) +
     p_i_throttle * m_ctl_err_int(0) + p_ff_throttle * m_controls(0,0));
     
  m_throttle_torque(1) = p_torque_smooth * m_throttle_torque(1) +
    (1 - p_torque_smooth) *
    (p_k_torque * m_ctl_err(1) + p_d_torque * m_ctl_err_vel(1) +
     p_i_torque * m_ctl_err_int(1) + p_ff_torque * m_controls(1,0));

}
    

// model predictive control using LQR for optimization
#define NUM_EXT_STATES (NUM_STATES+2*NUM_CONTROLS)

void MPCController::mpcLQR(const Matrix<double, NUM_STATES, 1> &s0,
			   const Matrix<double, NUM_CONTROLS, Dynamic> &u0,
			   const Matrix<double, NUM_STATES, Dynamic> &s_star,
			   const Matrix<double, NUM_CONTROLS, 1> &u_prev,
			   Matrix<double, NUM_CONTROLS, Dynamic> *u_out)
{
  Matrix<double, NUM_STATES, NUM_STATES> A[p_horizon];
  Matrix<double, NUM_STATES, NUM_CONTROLS> B[p_horizon];
  Matrix<double, NUM_EXT_STATES, NUM_EXT_STATES> Ae, P, Q;
  Matrix<double, NUM_EXT_STATES, NUM_CONTROLS> Be;
  Matrix<double, NUM_CONTROLS, NUM_EXT_STATES> K[p_horizon];
  Matrix<double, NUM_CONTROLS, 1> g[p_horizon], u_opt;
  Matrix<double, NUM_STATES, Dynamic> s(NUM_STATES, p_horizon+1);
  Matrix<double, NUM_EXT_STATES, 1> s_opt, q, ds;
  Matrix<double, NUM_STATES, 1> s_next;
  Matrix<double, NUM_CONTROLS, NUM_CONTROLS> Z;
  Matrix2d R;

  s.col(0) = s0;
  // initialize cost and dynamics matrices
  for (int i = 0; i < p_horizon; i++) {
    simulateRK4(s.col(i), u0.col(i), &s_next, &A[i], &B[i]);
    s.col(i+1) = s_next;
  }

  // initialize extended dynamics matrices
  Ae.setZero();
  Ae.block(NUM_STATES+NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) =
    Matrix<double,NUM_CONTROLS,NUM_CONTROLS>::Identity();
  Be.setZero();
  Be.block(NUM_STATES,0,NUM_CONTROLS,NUM_CONTROLS) =
    Matrix<double,NUM_CONTROLS,NUM_CONTROLS>::Identity();

  Q.setZero();
  Q.block(0,0,NUM_STATES,NUM_STATES) = p_Q;
  R = Rotation2D<double>(s_star(2,p_horizon)).toRotationMatrix();
  Q.block(0,0,2,2) = R.transpose() * p_Q.block(0,0,2,2) * R;
  Q.block(NUM_STATES, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES,
	  NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES, NUM_STATES + NUM_CONTROLS,
	  NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES + NUM_CONTROLS,
	  NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;

  s_opt.block(0,0,NUM_STATES,1) = s_star.col(p_horizon) - s.col(p_horizon);
  s_opt.block(NUM_STATES,0,NUM_CONTROLS,1) = -u0.col(p_horizon-1);
  s_opt.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1) = -u0.col(p_horizon-2);
  
  // Ricatti recursion  
  P = Q;
  q = -Q * s_opt;
  for (int i = p_horizon-1; i >= 0; i--) {
    R = Rotation2D<double>(s_star(2,i)).toRotationMatrix();
    Q.block(0,0,2,2) = R.transpose() * p_Q.block(0,0,2,2) * R;
    
    s_opt.block(0,0,NUM_STATES,1) = s_star.col(i) - s.col(i);
    if (i >= 1) {
      s_opt.block(NUM_STATES,0,NUM_CONTROLS,1) = -u0.col(i-1);
    } else {
      s_opt.block(NUM_STATES,0,NUM_CONTROLS,1) = -u_prev;
    }
    if (i >= 2) {
      s_opt.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1) = -u0.col(i-2);
    } else {
      s_opt.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1) = -u_prev;
    }
    u_opt = -u0.col(i);

    Ae.block(0,0,NUM_STATES,NUM_STATES) = A[i];
    Be.block(0,0,NUM_STATES,NUM_CONTROLS) = B[i];
    
    Z = (p_R + Be.transpose() * P * Be).inverse();
    K[i] = -Z * Be.transpose() * P * Ae;
    g[i] = -Z * (Be.transpose() * q - p_R * u_opt);
    P = Q + Ae.transpose() * P * Ae + Ae.transpose() * P * Be * K[i];
    P = 0.5*(P + P.transpose());
    q = (Ae+Be*K[i]).transpose() * q - K[i].transpose()*p_R*u_opt - Q*s_opt;
  }

  // simulate forward
  s_next = s0;
  for (int i = 0; i < p_horizon; i++) {
    ds.block(0,0,NUM_STATES,1) = s_next - s.col(i);
    if (i >= 1) {
      ds.block(NUM_STATES,0,NUM_CONTROLS,1) = u_out->col(i-1) - u0.col(i-1);
    } else {
      ds.block(NUM_STATES,0,NUM_CONTROLS,1).setZero();
    }
    if (i >= 2) {
      ds.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1) =
	u_out->col(i-2) - u0.col(i-2);
    } else {
      ds.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1).setZero();
    }
    
    u_out->col(i) = u0.col(i) + K[i]*ds + g[i];
    (*u_out)(1,i) = max((*u_out)(1,i), -p_vs.param.max_wheel_angle);
    (*u_out)(1,i) = min((*u_out)(1,i), p_vs.param.max_wheel_angle);
    simulateRK4(s_next, u_out->col(i), &s_next);
  }
}


#define EPSILON 1e-5

// dynamics of the car (bicycle model with velocity/steering input)
void MPCController::dynamics(const Matrix<double, NUM_STATES, 1> &s,
			     const Matrix<double, NUM_CONTROLS, 1> &u_,
			     Matrix<double, NUM_STATES, 1> *s_dot,
			     Matrix<double, NUM_STATES, NUM_STATES> *A,
			     Matrix<double, NUM_STATES, NUM_CONTROLS> *B)
{
  double u = s(3), v = s(4), cos_th = cos(s(2)), sin_th = sin(s(2));
  double th_dot = s(5), u_dot = u_(0);
  double del = u_(1), tan_del = tan(u_(1)), cos_del = cos(u_(1));
  double Fyf, Fyr, Ca = p_vs.param.tire_stiffness, m = p_vs.param.mass,
    a = p_vs.param.a, b = p_vs.param.b, I = p_vs.param.iz;

  // compute slip angles and lateral forces
  Fyf = -Ca * (atan2(v + th_dot * a, max(u,dgc_mph2ms(5))) - del);
  Fyr = -Ca * atan2(v - th_dot * b, max(u,dgc_mph2ms(5)));
  
  // compute derivatives
  (*s_dot)(0) = u * cos_th - v * sin_th;
  (*s_dot)(1) = u * sin_th + v * cos_th;
  (*s_dot)(2) = th_dot;

  (*s_dot)(3) = u_dot;
  (*s_dot)(4) = tan_del*(u_dot - th_dot*v) + (Fyf/cos_del + Fyr)/m - th_dot * u;
  (*s_dot)(5) = m*a/I*tan_del*(u_dot-th_dot*v) + a*Fyf/(I * cos_del) - b*Fyr/I;

  // compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      dynamics(s2, u_, &s_dot1, 0, 0);
      s2(i) -= 2*EPSILON;
      dynamics(s2, u_, &s_dot2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (s_dot1 - s_dot2) / (2*EPSILON);
    }
  }

  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u_;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      dynamics(s, u2, &s_dot1, 0, 0);
      u2(i) -= 2*EPSILON;
      dynamics(s, u2, &s_dot2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (s_dot1 - s_dot2) / (2*EPSILON);
    }
  }
}


void MPCController::simulateEuler(const Matrix<double, NUM_STATES, 1> &s,
				  const Matrix<double, NUM_CONTROLS, 1> &u,
				  Matrix<double, NUM_STATES, 1> *s_next,
				  Matrix<double, NUM_STATES, NUM_STATES> *A,
				  Matrix<double, NUM_STATES, NUM_CONTROLS> *B)
{
  Matrix<double, NUM_STATES, 1> s_dot;
  dynamics(s, u, &s_dot, A, B);
  (*s_next) = s + s_dot/p_hertz;

  if (A) {
    (*A) /= p_hertz;
    (*A) += Matrix<double, NUM_STATES, NUM_STATES>::Identity();
  }
  
  if (B) (*B) /= p_hertz;
}


void MPCController::simulateRK4(const Matrix<double, NUM_STATES, 1> &s,
				const Matrix<double, NUM_CONTROLS, 1> &u,
				Matrix<double, NUM_STATES, 1> *s_next,
				Matrix<double, NUM_STATES, NUM_STATES> *A,
				Matrix<double, NUM_STATES, NUM_CONTROLS> *B)
{
  Matrix<double, NUM_STATES, 1> k1, k2, k3, k4;
  double dt = 1 / p_hertz;

  dynamics(s, u, &k1);
  dynamics(s + 0.5*dt*k1, u, &k2);
  dynamics(s + 0.5*dt*k2, u, &k3);
  dynamics(s + dt*k3, u, &k4);
  (*s_next) = s + dt*(k1/6.0 + k2/3.0 + k3/3.0 + k4/6.0);


  // compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      simulateRK4(s2, u, &sn1, 0, 0);
      s2(i) -= 2*EPSILON;
      simulateRK4(s2, u, &sn2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (sn1 - sn2) / (2*EPSILON);
    }
  }

  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      simulateRK4(s, u2, &sn1, 0, 0);
      u2(i) -= 2*EPSILON;
      simulateRK4(s, u2, &sn2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (sn1 - sn2) / (2*EPSILON);
    }
  }
}
  


// use simulator to convert change in velocity to "throttle"
double MPCController::simulatorThrottle(double u, double v, double th_dot,
					double u_dot, double del)
{
  double Fyf, Fxf;
  double Ca = p_vs.param.tire_stiffness, m = p_vs.param.mass;

  Fyf = -Ca * (atan2(v + th_dot * p_vs.param.a, max(u,dgc_mph2ms(5))) - del);
  Fxf = (m*u_dot - m*th_dot*v + Fyf*sin(del)) / cos(del);

  if (Fxf > 0) {
    return Fxf / (m * p_vs.param.throttle_accel_coef);
  } else {
    return Fxf / (m * 100.0 * p_vs.param.brake_decel_coef);
  }
}  

} // namespace vlr
