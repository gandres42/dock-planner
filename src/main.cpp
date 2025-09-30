#include <stdio.h>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <ros/ros.h>
#include <open3d/Open3D.h>
#include <geometry_msgs/PoseStamped.h>
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
#define VISUALIZE true

class Planner {
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
    Planner() : grid(std::make_shared<open3d::geometry::VoxelGrid>()),
                dock_stl(std::make_shared<open3d::geometry::TriangleMesh>()),
                dock_nono_square_stl(std::make_shared<open3d::geometry::TriangleMesh>()),
                space(std::make_shared<ob::SE3StateSpace>()),
                si(nullptr),
                planner(nullptr)
    {
        open3d::io::ReadTriangleMesh("/home/gavin/ws/src/dock-planner/src/dock.stl", *dock_stl);
        open3d::io::ReadTriangleMesh("/home/gavin/ws/src/dock-planner/src/dock_nono_square.stl", *dock_nono_square_stl);
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

    void planPath(double start_x, double start_y, double start_z) {
        ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
        
        // Set start /goal states (position + rotation)
        start->rotation().setIdentity();
        goal->setXYZ(start_x, start_y, start_z);
        goal->rotation().setIdentity();
        start->setXYZ(0.0, 0.0, 0.0);
        std::shared_ptr<ob::ProblemDefinition> pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);
        planner->setProblemDefinition(pdef);
        ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(1.0));

        if (solved)
        {
            std::cout << "Found solution" << std::endl;
            auto path = pdef->getSolutionPath()->as<og::PathGeometric>();

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
        }
        else
        {
            std::cout << "No solution found." << std::endl;
        }
    }
};

int main() {
    Planner planner;
    planner.planPath(5, 0, 0);
    planner.planPath(-2, 2, 0);
    return 0;
}
