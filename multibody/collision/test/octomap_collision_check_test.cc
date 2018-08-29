#include "drake/multibody/collision/model.h"

#include "drake/common/find_resource.h"

#include <cmath>
#include <functional>
#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

// #include "drake/common/find_resource.h"
// #include "drake/common/test_utilities/eigen_matrix_compare.h"
// #include "drake/multibody/collision/drake_collision.h"

// #include "drake/multibody/collision/fcl_model.h"

// #include "drake/common/trajectories/piecewise_polynomial.h"

// #include <drake/math/roll_pitch_yaw.h>
// #include <drake/math/quaternion.h>
// #include <drake/math/rotation_matrix.h>

#include <fcl/fcl.h>

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;
using std::make_unique;
using std::move;
using std::unique_ptr;

using fcl::CollisionObject;
using fcl::CollisionGeometry;
using fcl::OcTree;

namespace drake {
namespace multibody {
namespace collision {
namespace {

/*

// Base fixture for tests that own a collision model
class OctomapCollisionChecker
{
 public:
    bool isFclCollisionFree( Eigen::VectorXd conf
                           , double collision_epsilon
    ) {
        KinematicsCache<double> cache = rigid_body_tree_->doKinematics(conf);
        rigid_body_tree_->updateDynamicCollisionElements(cache, true); 

        // bool bulletIsCollisionFree = checkCollisionsOnConfiguration(conf, collision_epsilon);

        drake::multibody::collision::FclModel robot_model;
        for (int j=0; j<num_actuatable_joints_; j++) {
            // make body_id string to pick correct joint from rigid_body_tree
            std::stringstream joint_name;
            joint_name << "iiwa_link_" << (j+1);
            // std::cout << "Body Index: " << rigid_body_tree_->FindBodyIndex(joint_name.str()) << std::endl;
            
            // Add the robot collision geometry for this configuration to our FclModel 
            FclAddElement(robot_model, joint_name.str());
        }
        // create an FCL collision manager for the robot
        FclSetRobot(robot_model.fcl_model_);

        fcl::test::CollisionData<double> collision_data;
        fcl::test::DistanceData<double> distance_data;
        collision_data.request.num_max_contacts = 100000;

        assert(robot_manager_); // Verify that we have successfully constructed a collision manager for the robot
        assert(obstacle_manager_ || octree_object_);
        // assert(fcl::test::defaultCollisionFunction);
        // std::cout << "ready to collide!\n";
        double min_distance = 1.0;
        if(obstacle_manager_ != nullptr){
            // std::cout << "obstacle_manager is defined!\n";
            robot_manager_->collide(obstacle_manager_, &collision_data, fcl::test::defaultCollisionFunction);
        // std::cout << "did we collide?\n";
            robot_manager_->distance(obstacle_manager_, &distance_data, fcl::test::defaultDistanceFunction);
            min_distance = distance_data.result.min_distance;
        }
        else if(octree_object_ != nullptr) {
            // std::cout << "octree_object_ is defined!\n";
            robot_manager_->octree_as_geometry_collide = true;
            robot_manager_->octree_as_geometry_distance = true;
            robot_manager_->collide(octree_object_, &collision_data, fcl::test::defaultCollisionFunction);
            // robot_manager_->distance(octree_object_, &distance_data, fcl::test::defaultDistanceFunction);
        }
        else{
            std::cout << "what???\n";
            return false;
        }
            
        if (!collision_data.result.isCollision() && min_distance > collision_epsilon) {
            if(!bulletIsCollisionFree)
                std::cout << "Bullet Collides! FCL does not collide! min distance: " << min_distance << std::endl;
            return true;
        }
        // std::cout << "Collides! numContacts: " << collision_data.result.numContacts() << std::endl;
        // std::cout << "Collides! min distance: " << min_distance << std::endl;
        return false;
    }

    void FclAddElement( drake::multibody::collision::FclModel &fclModel
                                        , std::string body_name
    ) {
        auto body = rigid_body_tree_->FindBody(body_name);
        auto collision_ids = body->get_mutable_collision_element_ids();
        for (auto &id : collision_ids) {
            auto collision_body = rigid_body_tree_->FindCollisionElement(id);
            // std::cout << *collision_body << std::endl;
            fclModel.DoAddElement(*collision_body);
        }
    }

    void FclSetRobot( std::vector<CollisionObject<double>*> &fcl_robot_model)
    {
        if (robot_manager_ != nullptr) {
            delete robot_manager_;
        }
        robot_manager_ = new fcl::DynamicAABBTreeCollisionManager<double>();
        robot_manager_->registerObjects(fcl_robot_model);
        robot_manager_->setup();
    }

    void FclSetObstacles(std::vector<CollisionObject<double>*> &fcl_obstacles)
    {
        if (octree_object_ != nullptr) {
            delete octree_object_;
        }
        if (obstacle_manager_ != nullptr) {
            delete obstacle_manager_;
        }
        obstacle_manager_ = new fcl::DynamicAABBTreeCollisionManager<double>();
        obstacle_manager_->registerObjects(fcl_obstacles);
        obstacle_manager_->setup();
    }

    void FclSetOcTreeObstacle(OcTree<double>* ocTree)
    {
        if (octree_object_ != nullptr) {
            delete octree_object_;
        }
        if (obstacle_manager_ != nullptr) {
            delete obstacle_manager_;
        }
        octree_object_ = new CollisionObject<double>(std::shared_ptr<CollisionGeometry<double>>(ocTree));
    }
 private:
    fcl::DynamicAABBTreeCollisionManager<double>* robot_manager_;
    fcl::DynamicAABBTreeCollisionManager<double>* obstacle_manager_;
    CollisionObject<double>* octree_object_; 
};

*/

void populateOcTree(octomap::OcTree* &ocTree, Eigen::Vector3d box_center, Eigen::Vector3d box_size, bool occupied, int num_per_side)
{
    // insert some measurements of occupied cells
    double x0 = box_center(0);
    double y0 = box_center(1);
    double z0 = box_center(2);
    double dx = box_size(0);
    double dy = box_size(1);
    double dz = box_size(2);
    for (int x = 0; x <= num_per_side; x++) {
        for (int y = 0; y <= num_per_side; y++) {
            for (int z = 0; z < num_per_side; z++) {
                double frac_x = double(x) / num_per_side - 0.5;
                double frac_y = double(y) / num_per_side - 0.5;
                double frac_z = double(z) / num_per_side - 0.5;
                ocTree->updateNode(octomap::point3d(dx*frac_x + x0, dy*frac_y + y0, dz*frac_z + z0), occupied);
            }
        }
    }
}

octomap::OcTree* generateBoxOcTree(double resolution)
{
    octomap::OcTree* ocTree = new octomap::OcTree(resolution);
    double dx = 0.0;
    double dy = 0.0;
    populateOcTree(ocTree, Eigen::Vector3d(0+dx, 0.8+dy, 0.4), Eigen::Vector3d(0.8, 0.8, 0.8), true, 20);
    populateOcTree(ocTree, Eigen::Vector3d(1.1+dx, 0.8+dy, 0.4), Eigen::Vector3d(0.8, 0.8, 0.8), true, 20);
    populateOcTree(ocTree, Eigen::Vector3d(-1.1+dx, 0.8+dy, 0.4), Eigen::Vector3d(0.8, 0.8, 0.8), true, 20);

    populateOcTree(ocTree, Eigen::Vector3d(0.55+dx, 0.8+dy, 0.4), Eigen::Vector3d(0.3, 0.8, 0.8), false, 20);
    populateOcTree(ocTree, Eigen::Vector3d(-0.55+dx, 0.8+dy, 0.4), Eigen::Vector3d(0.3, 0.8, 0.8), false, 20);

    std::cout << "This OcTree has " << ocTree->calcNumNodes() << " numNodes\n";
    std::cout << "This OcTree is at " << ocTree->getResolution() << " resolution\n";

    ocTree->writeBinary("simple_tree.bt");

    return ocTree;
}

// This tests that the query for distance between two objects (where at least
// one is non-convex) is gracefully handled -- no distance is reported.
GTEST_TEST(OctomapTest, CreateOctomap) {

    

    // 2. Create an OcTree which has the same occupancy as the boxes
    double resolution = 0.1; // meters
    OcTree<double>* ocTree = new OcTree<double>(std::shared_ptr<const octomap::OcTree>(generateBoxOcTree(resolution)));
    CollisionObject<double> tree_obj((std::shared_ptr<CollisionGeometry<double>>(ocTree)));

    EXPECT_TRUE(true); 

    std::cout << "this did something\n"; 
}

GTEST_TEST(OctomapTest, CreateOctomap) {
    std::string file_name = drake::FindResourceOrThrow(
        "drake/multibody/collision/test/ripple_cap.obj");

    int num_joints = 7;

    // 1. Test collision free planning using boxes
    Eigen::VectorXd conf_0 = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd conf_1(num_joints);
    Eigen::VectorXd conf_2(num_joints);
    conf_1 << 0.391309 ,     1.19062,     0.891649,    -0.863306,     0.694499,     0.512097, -0.000379773;
    conf_2 <<  -0.391345,     -1.19056,    -0.891491,     0.863343,    -0.694777,    -0.512052, -1.57062e-05;

    // 2. Create an OcTree which has the same occupancy as the boxes
    double resolution = 0.1; // meters
    OcTree<double>* ocTree = new OcTree<double>(std::shared_ptr<const octomap::OcTree>(generateBoxOcTree(resolution)));
    CollisionObject<double> tree_obj((std::shared_ptr<CollisionGeometry<double>>(ocTree)));




}



}  // namespace
}  // namespace collision
}  // namespace multibody
}  // namespace drake
