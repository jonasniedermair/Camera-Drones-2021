/**
* dynamicEDT3D:
* A library for incrementally updatable Euclidean distance transforms in 3D.
* @author C. Sprunk, B. Lau, W. Burgard, University of Freiburg, Copyright (C) 2011.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2011-2012, C. Sprunk, B. Lau, W. Burgard, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "util.h"

#include <ompl/base/SpaceInformation.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <geometry_msgs/Point.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include "ompl/util/Console.h"
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/base/StateValidityChecker.h>
#include <dla2_path_planner/helper_functions.h>
#include "visualization_msgs/MarkerArray.h"
#include <ompl/geometric/PathSimplifier.h>

#include <iostream>
#include <utility>

using namespace octomap;

visualization_msgs::Marker create_current_marker(double x, double y, double z) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}

void send_current_marker(ros::NodeHandle &nh, ros::Publisher &vis_pub, visualization_msgs::Marker &marker,
                         const geometry_msgs::Point &point) {
  vis_pub = nh.advertise<visualization_msgs::Marker>("marker_current", 0);
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = point;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  vis_pub.publish(marker);
}

void send_start_marker(ros::NodeHandle &nh, ros::Publisher &vis_pub, visualization_msgs::Marker &marker,
                       const geometry_msgs::Point &point) {
  vis_pub = nh.advertise<visualization_msgs::Marker>("marker_start", 0);
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = point;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub.publish(marker);
}

void send_end_marker(ros::NodeHandle &nh, ros::Publisher &vis_pub, visualization_msgs::Marker &marker,
                     const geometry_msgs::Point &point) {
  vis_pub = nh.advertise<visualization_msgs::Marker>("marker_end", 0);
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = point;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  vis_pub.publish(marker);
}

using namespace cd;

namespace ob = ompl::base;
namespace og = ompl::geometric;

template<class T, class U>
T conv(const U &p) {
    return T(p[0], p[1], p[2]);
}

class ValidityChecker3 : public ob::StateValidityChecker {
  const DynamicEDTOctomap *distmap;
  const OcTree *tree;
public:
  ValidityChecker3(const ob::SpaceInformationPtr &si,
                   const DynamicEDTOctomap *distmap,
                   const OcTree *tree) :
      ob::StateValidityChecker(si),
      distmap(distmap),
      tree(tree) {}

  // Returns whether the given state's position overlaps the
  // circular obstacle
  bool isValid(const ob::State *state) const override {
    return this->clearance(state) > 4 * tree->getResolution();
  }

  // Returns the distance from the given state's position to the
  // boundary of the circular obstacle.
  double clearance(const ob::State *state) const override {
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const auto *state3D =
        state->as<ob::RealVectorStateSpace::StateType>();

    // Extract the drone's (x, y, z) position from its state
    auto p = state3D->values;
    point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(conv<point3d>(p), distance, closestObst);
    return distance;
  }
};

struct DLA3PathPlanner {
  ros::NodeHandle &pnode_;
  ros::NodeHandle &node_;
  // parsed arguments
  double runTime;
  std::string outputFile;
  DynamicEDTOctomap *distmap;
  OcTree *tree;

  bool current_position_set{};
  geometry_msgs::Point current_position{}, goal_position{};
  bool traj_planning_successful{};
  std::shared_ptr<ompl::geometric::PathGeometric> p_last_traj_ompl;
  mav_planning_msgs::PolynomialTrajectory4D last_traj_msg;

  std::shared_ptr<ompl::geometric::PathGeometric> p_raw_last_traj_ompl;
  mav_planning_msgs::PolynomialTrajectory4D raw_last_traj_msg;
  bool bspline;
  bool perturb;


  DLA3PathPlanner(ros::NodeHandle &Pnode,
                  ros::NodeHandle &Node,
                  double RunTime,
                  std::string OutputFile,
                  DynamicEDTOctomap *distmap,
                  OcTree *tree)
      : pnode_(Pnode), node_(Node), runTime(RunTime),
        outputFile(std::move(OutputFile)), distmap(distmap),
        tree(tree) {
    current_position_sub = pnode_.subscribe("current_position", 10, &DLA3PathPlanner::currentPositionCallback, this);
    goal_position_sub = pnode_.subscribe("goal_position", 10, &DLA3PathPlanner::goalPositionCallback, this);
    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);
    trajectory_raw_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory_raw", 1);
    node_.param("bspline", bspline, true);
    node_.param("perturb", perturb, true);
    ROS_INFO_STREAM("bspline: " << bspline << " perturb: " << perturb);
  }

inline void plan() {
    // Construct the robot state space in which we're planning. We're
    // planning in [min,max]x[min,max]x[min,max], a subset of R^3.
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    double minX, minY, minZ;
    tree->getMetricMin(minX, minY, minZ);
    double maxX, maxY, maxZ;
    tree->getMetricMax(maxX, maxY, maxZ);

    // Set the bounds of space to be in [min, max].
    space->setBounds(std::min(std::min(minX, minY), minZ), std::max(std::max(maxX, maxY), maxZ));

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker3>(si, distmap, tree));

    si->setup();

    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_position.z;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_position.z;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    auto optimizationObjective(std::make_shared<ompl::base::MultiOptimizationObjective>(si));
    auto zPenal(std::make_shared<PathLengthOptimizationObjectiveZPenalized>(si));
    auto clearancePenal = std::make_shared<ClearanceObjective>(si);

    optimizationObjective->addObjective(zPenal, 1.0);
    optimizationObjective->addObjective(clearancePenal, 200.0);

    pdef->setOptimizationObjective(optimizationObjective);

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = std::make_shared<og::PRMstar>(si);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);

    if (solved) {
      // Output the length of the path found
      std::cout
          << optimizingPlanner->getName()
          << " found a solution of length "
          << pdef->getSolutionPath()->length()
          << " with an optimization objective value of "
          << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

      // If a filename was specified, output the path as a matrix to
      // that file for visualization
      if (!outputFile.empty()) {
        std::ofstream outFile(outputFile.c_str());
        std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
            printAsMatrix(outFile);
        outFile.close();
      }

      p_raw_last_traj_ompl = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());

      // 0.05 (less points)
      // 0.025 (quite good)
      // 0.01(more points)
      // original value: 0.005
      float snap_to_vertex = 0.025;

      int number_of_runs = 20;


      og::PathSimplifier simplifier(si, ompl::base::GoalPtr(), optimizationObjective);
    
      og::PathGeometric *path = new og::PathGeometric(*p_raw_last_traj_ompl); 
      double avg_costs = 0;
      ob::Cost original_cost = p_raw_last_traj_ompl->cost(optimizationObjective);

      for (int run = 0; run < number_of_runs; run++) {
        if (perturb) {
          simplifier.perturbPath(*path, 2.0, 100, 100, snap_to_vertex);
        }
        simplifier.shortcutPath(*path, 100, 100, 0.33, snap_to_vertex);
        avg_costs += path->cost(optimizationObjective).value();
      }

      p_last_traj_ompl = std::make_shared<ompl::geometric::PathGeometric>(*path);

      if (bspline) {
        simplifier.smoothBSpline(*path, 1);
      }
      avg_costs = avg_costs / number_of_runs;
      traj_planning_successful = true;

      std::vector<ompl::base::State*>& states = p_raw_last_traj_ompl->getStates();
      ob::StateValidityCheckerPtr checker = si->getStateValidityChecker();

      ROS_INFO_STREAM(std::endl << "Average cost: " << avg_costs << ", original cost: " << original_cost.value() << std::endl);

      delete path;
    } else {
      std::cout << "No solution found." << std::endl;
      traj_planning_successful = false;
    }
  }
  // ROS Topics
  ros::Subscriber current_position_sub;
  ros::Subscriber goal_position_sub;
  ros::Publisher trajectory_pub;
  ros::Publisher trajectory_raw_pub;



  void currentPositionCallback(const geometry_msgs::Point::ConstPtr &p_msg) {
    current_position_set = true;
    current_position = *p_msg;
    ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
  }

  void goalPositionCallback(const geometry_msgs::Point::ConstPtr &p_msg) {
    while (!current_position_set) {
      ROS_INFO_STREAM("Waiting for current position to be set");
      // Yields to something else.
      ros::spinOnce();
    }
    goal_position = *p_msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y << "; z: " << current_position.z);

    plan();

    if (traj_planning_successful) {
      sendLastMessage(last_traj_msg, p_last_traj_ompl);
      mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg =
          mav_planning_msgs::PolynomialTrajectory4D::Ptr(new mav_planning_msgs::PolynomialTrajectory4D(last_traj_msg));
      trajectory_pub.publish(p_traj_msg);

      sendLastMessage(raw_last_traj_msg, p_raw_last_traj_ompl);
      mav_planning_msgs::PolynomialTrajectory4D::Ptr p_raw_traj_msg =
          mav_planning_msgs::PolynomialTrajectory4D::Ptr(new mav_planning_msgs::PolynomialTrajectory4D(raw_last_traj_msg));
      // TODO: Is this correct?
      trajectory_raw_pub.publish(p_raw_traj_msg);
    }
  }

  inline void sendLastMessage(mav_planning_msgs::PolynomialTrajectory4D& trajectory, const std::shared_ptr<ompl::geometric::PathGeometric>& path) {
    mav_planning_msgs::PolynomialTrajectory4D &msg = trajectory;
    msg.segments.clear();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = path->getStates();
    for (auto p_s : states) {
      const double x = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
      const double y = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
      const double z = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
      const double yaw_s = 0.0;

      mav_planning_msgs::PolynomialSegment4D segment;
      segment.header = msg.header;
      segment.num_coeffs = 0;
      segment.segment_time = ros::Duration(0.);

      segment.x.push_back(x);
      segment.y.push_back(y);
      segment.z.push_back(z);
      segment.yaw.push_back(yaw_s);
      msg.segments.push_back(segment);
    }
  }
};

int main(int argc, char *argv[]) {
  if (argc <= 1) {
    std::cout << "usage: " << argv[0] << " <octoMap.bt>" << std::endl;
    exit(0);
  }

  auto *tree = new OcTree(0.05);

  //read in octotree
  if (!tree->readBinary(argv[1])) {
    std::cerr << "Unable to read octree given: " << argv[1] << std::endl;
    return 1;
  }

  double x, y, z;
  tree->getMetricMin(x, y, z);
  point3d min(x, y, z);
  tree->getMetricMax(x, y, z);
  point3d max(x, y, z);

  bool unknownAsOccupied = false;
  float maxDist = 1.0; // TODO: Figure out if we should configure this

  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  DynamicEDTOctomap distmap(maxDist, tree, min, max, unknownAsOccupied);

  // This computes the distance map.
  // If you modify the octree via tree->insertScan() or tree->updateNode(),
  // just call distmap.update() again to adapt the distance map to the changes made
  distmap.update();


  /* initialize ros */
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("Starting DLA3PathPlanner...");
  DLA3PathPlanner dla3_path_planner(nh, pnh, 1.0, "output", &distmap, tree);

  ROS_INFO("DLA3PathPlanner started...");

//  Eigen::Vector3d start_position{0, 0, 3};
//  Eigen::Vector3d end_position{10, -27, 15};
//
//  dla3_path_planner.current_position = eigToGeo(start_position);
//  dla3_path_planner.goal_position = eigToGeo(end_position);
//  dla3_path_planner.plan();
  ROS_INFO("DLA3PathPlanner finished");
  ros::Publisher vis_pub;
  visualization_msgs::Marker marker;

  while (true) {
//    dla3_path_planner.sendLastMessage();
    ros::spinOnce();
  }

  delete tree;

  return 0;
}
