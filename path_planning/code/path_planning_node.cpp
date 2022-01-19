
/* ---------------------------------------------------------------------------- 
   -------------------- Path Planning Module (EPM) - ARAV --------------------- 
   ----------------------------------------------------------------------------
   ------------------------ Author : Joan Bessa Sanz -------------------------- 
   ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- 
   ----------- (c) Copyright 2022. Joan Bessa. All Rights Reserved ------------  
   ---------------------------------------------------------------------------- */

//TODO pass octree resolution as an argument
//TODO set space bounds as arguments
//TODO pass start and end goals as arguments

/* Import required libraries */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
//#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <iostream>

#include <fcl/config.h>
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>

/* Namespaces */

namespace ob = ompl::base;
namespace og = ompl::geometric;

/* Topic names - (Global variables) */

/* ---------- INPUTS ---------- */

static const std::string INPUT_TOPIC_EPM_STATUS = "/arav/EPM/Status";
static const std::string INPUT_TOPIC_MAP = "/arav/octomap_binary";

/* ---------- OUTPUTS --------- */

static const std::string OUTPUT_TOPIC_PATH_LENGTH_GROUND = "/arav/path_planning/output/path_length_ground";
static const std::string OUTPUT_TOPIC_PATH_GROUND = "/arav/path_planning/output/path_ground";
static const std::string OUTPUT_TOPIC_VIS_GROUND = "/arav/path_planning/output/visualisation_ground";

static const std::string OUTPUT_TOPIC_PATH_LENGTH_AERIAL = "/arav/path_planning/output/path_length_aerial";
static const std::string OUTPUT_TOPIC_PATH_AERIAL = "/arav/path_planning/output/path_aerial";
static const std::string OUTPUT_TOPIC_VIS_AERIAL = "/arav/path_planning/output/visualisation_aerial";

/* ----- GLOBAL VARIABLES ----- */

static bool octomap_received = false;
static bool path_computed = false;

/* Definition of ROS Publishers */

ros::Publisher len_ground_pub;
ros::Publisher traj_ground_pub;
ros::Publisher vis_ground_pub;

ros::Publisher len_aerial_pub;
ros::Publisher traj_aerial_pub;
ros::Publisher vis_aerial_pub;

/* Definition of Path Planning Collision Model */

std::shared_ptr<fcl::CollisionGeometry> ARAV_Robot(new fcl::Box(2.5, 2.5, 2));
fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
fcl::CollisionObject treeObj((std::shared_ptr<fcl::CollisionGeometry>(tree)));
fcl::CollisionObject robotObject(ARAV_Robot);

/* State Validity Checker */

bool isStateValid(const ob::State *state)
{
    // Cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // Extract the first component (position) of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // Extract the second component (rotation) of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // Check validity of the state as defined by pos & rot
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	robotObject.setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());
}

/* Optimization Objective for Path Planning -> Path Length */

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	return obj;
}

/* Example of optimization objective with heuristics
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}
*/

/* Path Definition Function */

void plan(bool ground)
{
	// Check whether ground or aerial path must be computed
	double high_z_bound;
	if (ground)
	{
		high_z_bound = 0.1;
	}
	else
	{
		high_z_bound = 10;
	}

	// Create the state space for path planning
	ob::StateSpacePtr space(new ob::SE3StateSpace());

    // Set the bounds for the R^3 (translation) part of SE(3)
	ob::RealVectorBounds bounds(3);

	bounds.setLow(0,-50);
	bounds.setHigh(0,50);

	bounds.setLow(1,-50);
	bounds.setHigh(1,50);

	bounds.setLow(2,0.1);
	bounds.setHigh(2,high_z_bound);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // Construct an instance of Space Information from this State Space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Set State Validity Checking for this Space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

    // Create the start state (start position)
	ob::ScopedState<ob::SE3StateSpace> start(space);
	start->setXYZ(0,0,0.1);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // Create the goal state (goal position)
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	goal->setXYZ(-10,0,0.1);
	goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // Create problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // Set start and goal states
	pdef->setStartAndGoalStates(start, goal);

    // Create a planner for the defined space
	ob::PlannerPtr planner(new og::RRTstar(si));

    // Set the problem to be solved to the planner
	planner->setProblemDefinition(pdef);

    // Setup the planner
	planner->setup();

    // Print Space Settings
	si->printSettings(std::cout);

    // Print Problem Settings
	pdef->print(std::cout);

    // Attempt to solve problem in a definite amount of time
	ob::PlannerStatus solved = planner->solve(5); // Time in seconds


	std::cout << "Planner time finished." << std::endl;

	std_msgs::Float32 len_msg;

	if (solved)
	{
		// Get the solution from path planning
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
		std_msgs::Float64MultiArray msg;
		std::vector<double> point = { 0.0, 0.0, 0.0};
		//trajectory_msgs::MultiDOFJointTrajectory msg;
		//trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
		
		double length = path->length();
		len_msg.data = length;

		//msg.header.stamp = ros::Time::now();
		//msg.header.frame_id = "base_link";
		//msg.joint_names.clear();
		//msg.points.clear();
		//msg.joint_names.push_back("ARAV_Robot");

		/* -- Definition of visualisation marker -- */
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();
		marker.ns = "ARAV_Path";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.1; // Lines only need x scale (width)
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		if (ground)
		{
			marker.color.g = 1.0;
		}
		else
		{
			marker.color.b = 1.0;
		}
		/* ---------------------------------------- */

		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // Extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // Extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			//point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			//point_msg.transforms.resize(1);

			//point_msg.transforms[0].translation.x= pos->values[0];
			//point_msg.transforms[0].translation.y = pos->values[1];
			//point_msg.transforms[0].translation.z = pos->values[2];

			point[0] = -pos->values[0];
			point[1] = pos->values[1];
			point[2] = pos->values[2];
			msg.data = point;		

			// Publish individual point of the path
			if (ground) {traj_ground_pub.publish(msg);}
			else {traj_aerial_pub.publish(msg);}

			//point_msg.transforms[0].rotation.x = rot->x;
			//point_msg.transforms[0].rotation.y = rot->y;
			//point_msg.transforms[0].rotation.z = rot->z;
			//point_msg.transforms[0].rotation.w = rot->w;

			//msg.points.push_back(point_msg);

			/* -- Creation of visualisation marker -- */
			geometry_msgs::Point p;

			p.x = pos->values[0];
			p.y = pos->values[1];
			p.z = pos->values[2];
			marker.points.push_back(p);
			/* -------------------------------------- */
		}

		if (ground)
		{
			// Publish path length
			len_ground_pub.publish(len_msg);
			// Publish visualisation trajectory
			vis_ground_pub.publish(marker);
		}
		else
		{
			// Publish path length
			len_aerial_pub.publish(len_msg);	
			// Publish visualisation trajectory
			vis_aerial_pub.publish(marker);
		}
	}
	else
	{
		std::cout << "Path planning error: No solution found" << std::endl;
		
		// Publish "impossible path" length
		len_msg.data = -1;
		if (ground)
		{
			len_ground_pub.publish(len_msg);
		}
		else
		{
			len_aerial_pub.publish(len_msg);
		}
		
	}
}

void octomapCallback(const octomap_msgs::Octomap &msg)
{
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	octomap_received = true;
}

void statusCallback(const std_msgs::Int8 status)
{
	// function to activate path planning 
	if ((status.data == 1) && (!path_computed) && (octomap_received))
	{
		path_computed = true;
		// Ground path planning
		plan(true);
		// Aerial path planning
		plan(false);
	}

}

/* Main function */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle nh;
	ros::Subscriber octree_sub = nh.subscribe(INPUT_TOPIC_MAP, 1, octomapCallback);
	ros::Subscriber status_sub = nh.subscribe(INPUT_TOPIC_EPM_STATUS, 1, statusCallback);

	len_ground_pub = nh.advertise<std_msgs::Float32>(OUTPUT_TOPIC_PATH_LENGTH_GROUND,5);
	traj_ground_pub = nh.advertise<std_msgs::Float64MultiArray>(OUTPUT_TOPIC_PATH_GROUND,50);
	vis_ground_pub = nh.advertise<visualization_msgs::Marker>(OUTPUT_TOPIC_VIS_GROUND,5);

	len_aerial_pub = nh.advertise<std_msgs::Float32>(OUTPUT_TOPIC_PATH_LENGTH_AERIAL,5);
	traj_aerial_pub = nh.advertise<std_msgs::Float64MultiArray>(OUTPUT_TOPIC_PATH_AERIAL,50);
	vis_aerial_pub = nh.advertise<visualization_msgs::Marker>(OUTPUT_TOPIC_VIS_AERIAL,5);

	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}

//std_msgs/Float64MultiArray

/* ---------------------------------------------------------------------------- 
   -------------------- Path Planning Module (EPM) - ARAV --------------------- 
   ----------------------------------------------------------------------------
   ------------------------ Author : Joan Bessa Sanz -------------------------- 
   ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- 
   ----------- (c) Copyright 2021. Joan Bessa. All Rights Reserved ------------  
   ---------------------------------------------------------------------------- */