
/* ---------------------------------------------------------------------------- 
   -------------------- Path Planning Module (EPM) - ARAV --------------------- 
   ----------------------------------------------------------------------------
   ------------------------ Author : Joan Bessa Sanz -------------------------- 
   ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- 
   ----------- (c) Copyright 2022. Joan Bessa. All Rights Reserved ------------  
   ---------------------------------------------------------------------------- */

/* Import required libraries */

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

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

static const std::string INPUT_TOPIC_MAP = "/ARAV/octomap_binary";

/* ---------- OUTPUTS --------- */

static const std::string OUTPUT_TOPIC_PATH = "/ARAV/path_planning/output/path";
static const std::string OUTPUT_TOPIC_VIS = "/ARAV/path_planning/output/visualisation";

/* Definition of ROS Publishers */

ros::Publisher traj_pub;
ros::Publisher vis_pub;

/* Definition of Path Planning Collision Model */

std::shared_ptr<fcl::CollisionGeometry> ARAV_Robot(new fcl::Box(2, 2, 1));
fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
fcl::CollisionObject treeObj((std::shared_ptr<fcl::CollisionGeometry>(tree)));
fcl::CollisionObject aircraftObject(ARAV_Robot); // CHANGE TO robotObject()

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
	aircraftObject.setTransform(rotation, translation); // CHANGE TO robotObject()
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult); // CHANGE TO robotObject()

	return(!collisionResult.isCollision());
}

/* Optimization Objective for Path Planning -> Path Length */

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	return obj;
}

/* NECESSARY?
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}
*/

/* Path Definition Function */

void plan(void)
{
	// Create the state space for path planning
	ob::StateSpacePtr space(new ob::SE3StateSpace());

    // Set the bounds for the R^3 (translation) part of SE(3)
	ob::RealVectorBounds bounds(3);

	bounds.setLow(0,-50);
	bounds.setHigh(0,50);

	bounds.setLow(1,-50);
	bounds.setHigh(1,50);

	bounds.setLow(2,0.1);
	bounds.setHigh(2,0.1);

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
	goal->setXYZ(-8,0,0.1);
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
	ob::PlannerStatus solved = planner->solve(1.0); // Time in seconds


	std::cout << "Planner time finished." << std::endl;

	if (solved)
	{
		// Get the solution from path planning
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);

        // print the path to screen
        // path->print(std::cout);

		trajectory_msgs::MultiDOFJointTrajectory msg;
		trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "BaseLink";
		msg.joint_names.clear();
		msg.points.clear();
		msg.joint_names.push_back("ARAV_Robot");

		/* -- Definition of visualisation marker -- */
		visualization_msgs::Marker marker;
		marker.header.frame_id = "BaseLink";
		marker.header.stamp = ros::Time::now();
		marker.ns = "ARAV_Path";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.1; // Lines only need x scale (width)
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		/* ---------------------------------------- */

		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // Extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // Extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			point_msg.transforms.resize(1);

			point_msg.transforms[0].translation.x= pos->values[0];
			point_msg.transforms[0].translation.y = pos->values[1];
			point_msg.transforms[0].translation.z = pos->values[2];

			point_msg.transforms[0].rotation.x = rot->x;
			point_msg.transforms[0].rotation.y = rot->y;
			point_msg.transforms[0].rotation.z = rot->z;
			point_msg.transforms[0].rotation.w = rot->w;

			msg.points.push_back(point_msg);

			/* -- Creation of visualisation marker -- */
			geometry_msgs::Point p;

			p.x = pos->values[0];
			p.y = pos->values[1];
			p.z = pos->values[2];
			marker.points.push_back(p);
			/* -------------------------------------- */
		}

		// Publish resulting trajectory
		traj_pub.publish(msg);

		/* -- Publish visualisation marker -- */
		vis_pub.publish(marker);
		/* ----------------------------------- */
	}
	else
		std::cout << "Path planning error: No solution found" << std::endl;
}

void octomapCallback(const octomap_msgs::Octomap &msg)
{
    // Load collision OcTree from Octomap Message
	//octomap::OcTree* temp_tree = new octomap::OcTree(0.1);
	//octomap::AbstractOcTree* abs_tree = octomap_msgs::fullMsgToMap(msg);
	//temp_tree = dynamic_cast<octomap::OcTree*>(abs_tree);

	/*octomap::OcTree temp_tree = octomap_msgs::fullMsgToMap(msg);*/

	//fcl::OcTree tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	// convert octree to collision object
	
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	//fcl::CollisionObject treeObj = ((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	plan();

}

/* Main function */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle n;
	ros::Subscriber octree_sub = n.subscribe(INPUT_TOPIC_MAP, 1, octomapCallback);
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(OUTPUT_TOPIC_PATH,10);
	vis_pub = n.advertise<visualization_msgs::Marker>(OUTPUT_TOPIC_VIS, 0 );

	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}

/* ---------------------------------------------------------------------------- 
   -------------------- Path Planning Module (EPM) - ARAV --------------------- 
   ----------------------------------------------------------------------------
   ------------------------ Author : Joan Bessa Sanz -------------------------- 
   ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- 
   ----------- (c) Copyright 2021. Joan Bessa. All Rights Reserved ------------  
   ---------------------------------------------------------------------------- */