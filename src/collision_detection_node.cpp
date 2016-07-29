#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
using octomap_msgs::GetOctomap;

#include <collision_detection/CheckCollision.h>
#include <collision_detection/test_fcl_utility.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/collision.h"
#include "fcl/octree.h"
#include "fcl/math/transform.h"
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

#include <sstream>
#include <boost/shared_ptr.hpp>
using namespace octomap;
using namespace fcl;

class CollisionDetector{
public:

    ros::NodeHandle node;
    geometry_msgs::PoseArray p_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    ros::Publisher visCubePub;
    ros::ServiceServer service;

    CollisionDetector(ros::NodeHandle n):node(n)
    {
        ROS_INFO("Setting up collision detection node ...");

        visCubePub = node.advertise<visualization_msgs::MarkerArray>("Cube", 1);
        service = node.advertiseService("path_env_collision", &CollisionDetector::checkCollision, this);

        ROS_INFO("Ready to detect collision.");

        ros::Rate loop_rate(1);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool checkCollision(collision_detection::CheckCollision::Request  &req, collision_detection::CheckCollision::Response &res)
    {
        ROS_INFO("Get a collision detection request ...");
        
        ROS_INFO("Requesting the map from %s...", node.resolveName("octomap_binary").c_str());
        GetOctomap::Request octomap_req;
        GetOctomap::Response octomap_resp;
        while(node.ok() && !ros::service::call("octomap_binary", octomap_req, octomap_resp))
        {
            ROS_WARN("Request to %s failed; trying again...", node.resolveName("octomap_binary").c_str());
            usleep(1000000);
        }

        if (node.ok())
        { // skip when CTRL-C
            AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_resp.map);
            ROS_INFO("Got map");

            octomap::OcTree* octree = NULL;
            if (tree)
            {
                octree = dynamic_cast<octomap::OcTree*>(tree);
            } 
            else
            {
                ROS_ERROR("Error creating octree from received message");
                if (octomap_resp.map.id == "ColorOcTree")
                ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
            }

            fcl::OcTree* tree2 = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octree));
            std::vector<CollisionObject*> boxes;
            generateBoxesFromOctomap(boxes, *tree2);


            BroadPhaseCollisionManager* manager_map = new DynamicAABBTreeCollisionManager();
            manager_map->registerObjects(boxes);

            p_array = req.path;
            BroadPhaseCollisionManager* manager_path = new DynamicAABBTreeCollisionManager();
            for(std::size_t i = 0; i < p_array.poses.size(); ++i)
            {
                boost::shared_ptr <fcl::CollisionGeometry> boxGeometry (new fcl::Box (0.1, 0.1, 0.1));
                geometry_msgs::Pose pt = p_array.poses[i];
                fcl::Transform3f tf1 (fcl::Vec3f (pt.position.x, pt.position.y, pt.position.z));
                fcl::CollisionObject* box = new CollisionObject(boxGeometry, tf1);
                manager_path->registerObject(box);
            }

            DistanceData distance_data;
            manager_map->setup();
            manager_path->setup();
            manager_path->distance(manager_map, &distance_data, defaultDistanceFunction);
            FCL_REAL min_distance = distance_data.result.min_distance;
            ROS_INFO("The minimum distance is %f", min_distance);
            if (min_distance == -1.0)
            {
                res.collide = true;
            }
            else
            {
                res.collide = false;
            }
        }

        return true;
    }


    visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/camera_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;//poseQ[1];
        marker.pose.orientation.z = 0;//poseQ[2];
        marker.pose.orientation.w = 1;//poseQ[3];

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
        }
        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;
        }
        marker.lifetime = ros::Duration(0.3);
        return marker ;
    }

    void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, fcl::OcTree& tree)
    {
        ROS_INFO("Converting tree to box...");
        std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];
            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            CollisionObject* obj = new CollisionObject(boost::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.push_back(obj);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detection_node");
    ros::NodeHandle n;
    CollisionDetector collision_detector(n);
    return 0;
}
