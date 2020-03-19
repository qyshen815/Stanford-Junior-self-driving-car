/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_StDrivePassObstacle.hpp"
#include "aw_StPause.hpp"
#include "aw_StError.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrivePassObstacle
 *---------------------------------------------------------------------------*/
StDrivePassObstacle::StDrivePassObstacle(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstacle"))
 ,lcm(0), pom(0)
{
	lcm = new LaneChangeManager(context<ChsmPlanner>().topology, context<ChsmPlanner>().topology->vehicle_manager);
	pom = new PassObstacleManager(context<ChsmPlanner>().topology);
}

StDrivePassObstacle::~StDrivePassObstacle() {
	delete lcm;
	delete pom;
}

sc::result StDrivePassObstacle::react(const EvProcess&) {
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDrivePassObstacleBranch
 *---------------------------------------------------------------------------*/
StDrivePassObstacleBranch::StDrivePassObstacleBranch(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstacleBranch"))
{
	decision_time.now();
	decision_time += 1.0;
}

StDrivePassObstacleBranch::~StDrivePassObstacleBranch() {
}

sc::result StDrivePassObstacleBranch::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();
	double st_vh_dist, mv_vh_dist;
	planner.getVehicleDistances(st_vh_dist, mv_vh_dist);

	if (mv_vh_dist < st_vh_dist) { // TODO: maybe delay this transition so that mis-readings don't result in immediate transitions
		return transit<StDrive>();
	}

	// try to init manager with vehicle
	if (!parent.pom->isInited()) {
		Vehicle* obstacle = planner.topology->get_next_vehicle();
		if (obstacle) {
			parent.pom->setObstacle(obstacle);
		}
	}
	parent.pom->update();

	// Transition: Stop (if we could not make the decision to change lanes in 1 second)
	if (isExpired(decision_time)) {
		return transit<StDrivePassObstacleStop>();
	}

	// TODO: LaneChangeManager: see if we can pass the obstacle

	planner.generateCurvePoints(planner.params().max_speed_drive); // curvepoints stop in front of obstacle
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDrivePassObstacleStop
 *---------------------------------------------------------------------------*/
StDrivePassObstacleStop::StDrivePassObstacleStop(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstacleStop"))
{
}

StDrivePassObstacleStop::~StDrivePassObstacleStop() {
}

sc::result StDrivePassObstacleStop::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	Vehicle* obstacle = planner.topology->get_next_vehicle();
	if (!parent.pom->isInited() && obstacle) {
		parent.pom->setObstacle(obstacle);
	}
	parent.pom->update();

	double st_vh_dist, mv_vh_dist;
	planner.getVehicleDistances(st_vh_dist, mv_vh_dist);

	if (mv_vh_dist < st_vh_dist) { // TODO: maybe delay this transition so that mis-readings don't result in immediate transitions
		return transit<StDrive>();
	}

	if (st_vh_dist < STD_VEHICLE_LENGTH && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
		return transit<StDrivePassObstacleWait>();
	}

	// is done in St...Wait - so car has to stop before doing recover
	//if (!parent.pom->isPassPossible()) {
	//	return transit<StDriveRecover>();
	//}

	planner.generateCurvePoints(planner.params().max_speed_pass_obstacle); // curvepoints stop in front of obstacle
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDrivePassObstacleWait
 *---------------------------------------------------------------------------*/
StDrivePassObstacleWait::StDrivePassObstacleWait(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstacleWait"))
{
	waitTimeout.now();
	waitTimeout += MIN_WAIT_OBSTACLE;

	congestionTimeout.now();
	congestionTimeout += CONGESTION_TIMEOUT;
}

StDrivePassObstacleWait::~StDrivePassObstacleWait() {
}

sc::result StDrivePassObstacleWait::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	if (!parent.pom->isInited()) {
		Vehicle* obstacle = planner.topology->get_next_vehicle();
		if (obstacle) {
			parent.pom->setObstacle(obstacle);
		}
	}
	if (!parent.pom->isInited()) {
		planner.addMessage("PassObstacleManager has no valid obstacle to pass");
		return transit<StError>();
	}
	parent.pom->update();

	double st_vh_dist, mv_vh_dist;
	planner.getVehicleDistances(st_vh_dist, mv_vh_dist);

	// Transition: obstacle moves again
	if (mv_vh_dist < st_vh_dist) { // TODO: maybe delay this transition so that mis-readings don't result in immediate transitions
		return transit<StDrive>();
	}

	if (parent.pom->mayPass(planner.currentPose(), planner.currentPose().v(), 2.0) && isExpired(waitTimeout)) {
		return transit<StDrivePassObstaclePass>();
	}

	// Transition: recover from failure
	//if (isExpired(congestionTimeout) || !parent.pom->isPassPossible()) {
	//	return transit<StDriveRecover>();
	//}

	//planner.generateCurvePoints(0.0, std::numeric_limits<double>::max()); // TODO: what's the best way to stop the car?
	planner.velocity_desired_ = 0.5; // TODO: ?!?
	return forward_event();
}

/*---------------------------------------------------------------------------
 * StDrivePassObstaclePass
 *---------------------------------------------------------------------------*/
StDrivePassObstaclePass::StDrivePassObstaclePass(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstaclePass"))
{

}

StDrivePassObstaclePass::~StDrivePassObstaclePass() {
}

sc::result StDrivePassObstaclePass::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	if (!parent.pom->isInited()) {
		Vehicle* obstacle = planner.topology->get_next_vehicle();
		if (obstacle) {
			parent.pom->setObstacle(obstacle);
		}
	}
	if (!parent.pom->isInited()) {
		planner.addMessage("PassObstacleManager has no valid obstacle to pass");
		return transit<StError>();
	}
	parent.pom->update();

	if (!parent.pom->mayPass(planner.currentPose(), planner.currentPose().v(), 2.0)) {
//		if (!parent.pom->mayPass(planner.currentPose().v(), 2.0)) {
		std::cerr << "While passing an obstacle, PassObstacleManager doesn't allow the pass anymore. ignore this and go on..." << std::endl;
	}

	// TODO: implement and check
  double front_sample_length = std::max(planner.params().center_line_min_lookahead_dist, planner.traj_eval_->params().checked_horizon * planner.params().max_speed_pass_obstacle);
  parent.pom->generateTrajectory(planner.currentPose(), planner.center_line_, front_sample_length, planner.params().center_line_back_sample_length);

  // calculate desired velocity based on speed limits and curvature
  // TODO: make this work again..
//  planner.velocity_desired_ = planner.calculateVelocity(planner.params().max_speed_pass_obstacle, front_sample_length);  // TODO: correct front sample length
//	planner.velocity_following_ = parent.pom->getFollowingSpeed();

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StDrivePassObstacleChangeLanes
 *---------------------------------------------------------------------------*/
StDrivePassObstacleChangeLanes::StDrivePassObstacleChangeLanes(my_context ctx) : my_base(ctx), kogmo_base(std::string("StDrivePassObstacleChangeLanes"))
{
}

StDrivePassObstacleChangeLanes::~StDrivePassObstacleChangeLanes() {
}

sc::result StDrivePassObstacleChangeLanes::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

//	ChsmPlanner& planner = context<ChsmPlanner>();
//	StDrivePassObstacle& parent = context<StDrivePassObstacle>();
	return forward_event();
	// TODO: implement
}

} // namespace vlr
