#include <stdio.h>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <open3d/Open3D.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>
#include <iostream>
#include <Eigen/Dense>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define VOXEL_RESOLUTION 0.01
#define VISUALIZE false
#define HORIZON_DISTANCE 0.5

class RRTPlanner {
private:
    std::shared_ptr<open3d::geometry::VoxelGrid> grid;
    std::shared_ptr<open3d::geometry::TriangleMesh> dock_stl;
    std::shared_ptr<open3d::geometry::TriangleMesh> dock_nono_square_stl;
    std::shared_ptr<ob::SE3StateSpace> space;
    std::shared_ptr<ob::SpaceInformation> si;
    std::shared_ptr<og::RRT> planner;

    bool stateValidityChecker(const ob::State *state) {
        const auto *se3 = state->as<ob::SE3StateSpace::StateType>();
        Eigen::Vector3d point(se3->getX(), se3->getY(), se3->getZ());
        std::vector<Eigen::Vector3d> query = {point};
        std::vector<bool> query_results = this->grid->CheckIfIncluded(query);
        return !query_results[0];
    }

public:
    RRTPlanner() : grid(std::make_shared<open3d::geometry::VoxelGrid>()),
                dock_stl(std::make_shared<open3d::geometry::TriangleMesh>()),
                dock_nono_square_stl(std::make_shared<open3d::geometry::TriangleMesh>()),
                space(std::make_shared<ob::SE3StateSpace>()),
                si(nullptr),
                planner(nullptr)
    {
        open3d::io::ReadTriangleMesh("/home/gavin/ws/src/dock-planner/src/dock.stl", *dock_stl);
        open3d::io::ReadTriangleMesh("/home/gavin/ws/src/dock-planner/src/dock_nono_square.stl", *dock_nono_square_stl);

        // Rotate both meshes by 180 degrees around Z axis
        Eigen::AngleAxisd rot_z(M_PI, Eigen::Vector3d::UnitZ());
        dock_stl->Rotate(rot_z.toRotationMatrix(), Eigen::Vector3d(0, 0, 0));
        dock_nono_square_stl->Rotate(rot_z.toRotationMatrix(), Eigen::Vector3d(0, 0, 0));

        grid = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*dock_nono_square_stl, VOXEL_RESOLUTION);

        // SE3 state space - position bounds only (orientation is handled separately)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(-5);
        bounds.setHigh(5);
        space->setBounds(bounds);

        // Space information
        si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([this](const ob::State *state) {
            return this->stateValidityChecker(state);
        });
        si->setup();

        // geometric planner
        planner = std::make_shared<og::RRT>(si);
        planner->setRange(0.01);
        planner->setup();
    }

    std::shared_ptr<og::PathGeometric> planPath(double start_x, double start_y, double start_z) {
        ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
        
        // Set start /goal states (position + rotation)
        start->rotation().setIdentity();
        goal->setXYZ(start_x, start_y, start_z);
        goal->rotation().setIdentity();
        start->setXYZ(0.0, 0.0, 0.0);
        std::shared_ptr<ob::ProblemDefinition> pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);
        planner->clear();
        planner->setProblemDefinition(pdef);
        ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(1.0));

        std::shared_ptr<og::PathGeometric> path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (!path) {
            std::cerr << "Failed to cast solution path to PathGeometric." << std::endl;
            return nullptr;
        }
        if (VISUALIZE) {
            std::vector<Eigen::Vector3d> points;
            auto path_states = path->getStates();
            for (const auto *state : path_states) {
                const auto *se3 = state->as<ob::SE3StateSpace::StateType>();
                points.emplace_back(se3->getX(), se3->getY(), se3->getZ());
            }

            // Create line set for path
            auto line_set = std::make_shared<open3d::geometry::LineSet>();
            for (const auto &pt : points) {
                line_set->points_.push_back(pt);
            }
            for (size_t i = 1; i < points.size(); ++i) {
                line_set->lines_.push_back(Eigen::Vector2i(i - 1, i));
            }
            line_set->colors_.resize(line_set->lines_.size(), Eigen::Vector3d(1, 0, 0)); // Red lines
            
            open3d::visualization::DrawGeometries({line_set, dock_stl}, "Path Visualization");
        }
        return path;
    }
};

class Docking {
private:
    ros::NodeHandle n;
    RRTPlanner p;
    ros::Subscriber pose_sub;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;

    std::tuple<double, double, double> prev_plan_pose;
    std::shared_ptr<og::PathGeometric> path;

    double pointDistance(std::tuple<double, double, double> pos_a, std::tuple<double, double, double> pos_b) {
        double dx = std::get<0>(pos_a) - std::get<0>(pos_b);
        double dy = std::get<1>(pos_a) - std::get<1>(pos_b);
        double dz = std::get<2>(pos_a) - std::get<2>(pos_b);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::tuple<double, double, double> firstPoint(std::shared_ptr<og::PathGeometric> path_ptr, std::tuple<double, double, double> pose, double dist) {
        if (path_ptr) {
            for (const auto *state : path_ptr->getStates()) {
                const auto *se3 = state->as<ob::SE3StateSpace::StateType>();
                std::tuple<double, double, double> pt(se3->getX(), se3->getY(), se3->getZ());
                if (this->pointDistance(pt, pose) <= dist) {
                    return pt;
                }
            }
        }

        return std::make_tuple(
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()
        );
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double px = msg->pose.position.x;
        double py = msg->pose.position.y;
        double pz = msg->pose.position.z;
        if (this->pointDistance(this->prev_plan_pose, std::make_tuple(px, py, pz)) > HORIZON_DISTANCE) {
            ROS_INFO("Path replan triggered");
            this->path = this->p.planPath(px, py, pz);
            this->prev_plan_pose = std::make_tuple(px, py, pz);

            if (this->path) {
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";
                for (const auto *state : this->path->getStates()) {
                    const auto *se3 = state->as<ob::SE3StateSpace::StateType>();
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header = path_msg.header;
                    pose_stamped.pose.position.x = se3->getX();
                    pose_stamped.pose.position.y = se3->getY();
                    pose_stamped.pose.position.z = se3->getZ();
                    pose_stamped.pose.orientation.x = se3->rotation().x;
                    pose_stamped.pose.orientation.y = se3->rotation().y;
                    pose_stamped.pose.orientation.z = se3->rotation().z;
                    pose_stamped.pose.orientation.w = se3->rotation().w;
                    path_msg.poses.push_back(pose_stamped);
                }
                this->path_pub.publish(path_msg);
            }
        }

        std::tuple<double, double, double> nearest_pt = this->firstPoint(this->path, std::make_tuple(px, py, pz), HORIZON_DISTANCE);
        geometry_msgs::PoseStamped nearest_pose_msg;
        nearest_pose_msg.header.stamp = ros::Time::now();
        nearest_pose_msg.header.frame_id = "map";
        nearest_pose_msg.pose.position.x = std::get<0>(nearest_pt);
        nearest_pose_msg.pose.position.y = std::get<1>(nearest_pt);
        nearest_pose_msg.pose.position.z = std::get<2>(nearest_pt);
        nearest_pose_msg.pose.orientation.w = 1.0;
        nearest_pose_msg.pose.orientation.x = 0.0;
        nearest_pose_msg.pose.orientation.y = 0.0;
        nearest_pose_msg.pose.orientation.z = 0.0;
        this->pose_pub.publish(nearest_pose_msg);
    }
public:
    Docking() {
        pose_sub = n.subscribe("/aruco/pose", 1, &Docking::poseCallback, this);
        path_pub = n.advertise<nav_msgs::Path>("/sianat/path", 1);
        pose_pub = n.advertise<geometry_msgs::PoseStamped>("/sianat/pose", 1);
        prev_plan_pose = std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
        path = nullptr;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "planner");
    Docking d;
    ros::spin();
    return 0;
}
