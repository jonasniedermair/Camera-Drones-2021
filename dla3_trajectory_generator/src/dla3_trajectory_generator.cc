#include <dla3_trajectory_generator/dla3_trajectory_generator.h>

#include <Eigen/Dense>

DLA3TrajectoryGenerator::DLA3TrajectoryGenerator(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(2.0),
    max_a_(2.0),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()),
    raw(false) {
      
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[dla3_trajectory_generator] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[dla3_trajectory_generator] param max_a not found");
  }
  if (!nh.getParam("raw", raw)) {
    ROS_WARN("[dla3_trajectory_generator] param raw not found");
  }


  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("/trajectory_generator/smooth_trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("/trajectory_generator/smooth_trajectory",
                                                              0);

  // subscriber for Odometry
   sub_odom_ =
       nh.subscribe("uav_pose", 1, &DLA3TrajectoryGenerator::uavOdomCallback, this);

  // subscriber for trajectory
  ROS_INFO("Subscribe to /path_planner/planned_trajectory");
  sub_trajectory_ = nh_.subscribe("/path_planner/planned_trajectory" + std::string(raw ? "_raw" : ""), 10, &DLA3TrajectoryGenerator::trajectoryCallback, this);
    
}

void DLA3TrajectoryGenerator::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D msg)
{
  ROS_INFO("Trajectory message received!");
  
  // Get trajectory by message
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(msg, &trajectory);

  // Get trajectory segments
  mav_trajectory_generation::Segment::Vector segments;
  trajectory.getSegments(&segments);

  std::vector<Eigen::VectorXd> positions;

  // Get positions
  for (int i = 0; i < segments.size(); i++)
  {
    Eigen::VectorXd tmp(3);
    for (int j = 0; j < 3; ++j)
      tmp(j) = segments[i][j].getCoefficients(mav_trajectory_generation::derivative_order::POSITION)(0);
    positions.push_back(tmp);
  }
  
  // Plan & publish trajectory
  ROS_INFO("Plan trajectory");
  planTrajectory(positions, &trajectory);
   
  ROS_INFO("Publish trajectory");
  publishTrajectory(trajectory);
}

// Callback to get current Pose of UAV
void DLA3TrajectoryGenerator::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void DLA3TrajectoryGenerator::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

bool DLA3TrajectoryGenerator::computeMaxJerkAndSnap(mav_trajectory_generation::Trajectory *trajectory, double *jerk, double *snap) {
  std::vector<int> dimensions(trajectory->D());  // Evaluate in whatever dimensions we have.
  std::iota(dimensions.begin(), dimensions.end(), 0);

  mav_trajectory_generation::Extremum v_min_traj, v_max_traj, a_min_traj, a_max_traj;

  bool success = trajectory->computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::JERK, dimensions,
      &v_min_traj, &v_max_traj);
  success &= trajectory->computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::SNAP, dimensions,
      &a_min_traj, &a_max_traj);

  *jerk = v_max_traj.value;
  *snap = a_max_traj.value;
  return success;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool DLA3TrajectoryGenerator::planTrajectory(const std::vector<Eigen::VectorXd> positions, mav_trajectory_generation::Trajectory* trajectory) {
  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 3;

  // Get start and goal position
  auto start_position = positions[0];
  auto goal_position = positions.back();

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // end = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);


  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(start_position, derivative_to_optimize);

  // Set start point velocity to 0.0 (hover state)
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0.0);

  // add waypoint to list
  vertices.push_back(start);

  /******* Configure points between start and end point *******/
  for (int i = 1; i < positions.size() - 1; i++)
  {
    mav_trajectory_generation::Vertex point_between(dimension);
    point_between.addConstraint(mav_trajectory_generation::derivative_order::POSITION, positions[i]);
    vertices.push_back(point_between);
  }

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_position, derivative_to_optimize);


  // set goal point velocity to 0.0 (hover state)
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0.0);

  // add waypoint to list
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // Set non-linear optimization parameters
  parameters.algorithm = nlopt::LN_SBPLX;
  parameters.f_rel = 0.00005;
  parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTime;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));

  auto result = opt.getOptimizationInfo();

  double estimated_time = trajectory->getMaxTime();
  double estimated_v, estimated_a;
  double estimated_j, estimated_s;
  trajectory->computeMaxVelocityAndAcceleration(&estimated_v, &estimated_a);
  computeMaxJerkAndSnap(trajectory, &estimated_j, &estimated_s);

  std::cout << "Estimated time: " << estimated_time << std::endl;
  std::cout << "Estimated velocity: " << estimated_v << std::endl;
  std::cout << "Estimated acceleration: " << estimated_a << std::endl;
  std::cout << "Estimated jerk: " << estimated_j << std::endl;
  std::cout << "Estimated snap: " << estimated_s << std::endl;

  return true;
}

bool DLA3TrajectoryGenerator::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}
