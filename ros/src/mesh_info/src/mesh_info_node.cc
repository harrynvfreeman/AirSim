#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

//Airsim/Unreal
#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
//

//using namespace msr::airlib;
using namespace sensor_msgs;

class MeshInfoNode {
 public:
  MeshInfoNode();
  ~MeshInfoNode();

 private:
  void getMeshVertices(const ros::TimerEvent &e);
  ros::Publisher mesh_vertex_pub_;
  ros::Timer publishing_timer_;
  double publish_freq_;
};

MeshInfoNode::MeshInfoNode() {
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  if (!priv_nh.getParam("publish_freq", publish_freq_)) { publish_freq_ = 50.0; }

  mesh_vertex_pub_ = nh.advertise<sensor_msgs::PointCloud2>("mesh_vertices", 1);
  publishing_timer_ = nh.createTimer(ros::Duration(1 / publish_freq_ /2.0), &MeshInfoNode::getMeshVertices, this);
}

MeshInfoNode::~MeshInfoNode() {
}

void MeshInfoNode::getMeshVertices(const ros::TimerEvent &e) {
    msr::airlib::MultirotorRpcLibClient client;
    //client.confirmConnection();

    auto meshes = client.simGetSkeletalMeshPositionVertexBuffers();

    uint8_t *uint_array;
    for(auto &mesh : meshes) {
    if (mesh.name == "person_actor" && mesh.ownerHasOwner) {
        PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
        int numVertices = mesh.vertices.size() / 3;
        points_msg->header.frame_id = "vertices";
        points_msg->height = 1;
        points_msg->width = numVertices;
        points_msg->is_bigendian = false;
        //points_msg->is_dense = true; //or should be false?
        points_msg->is_dense = false; //or should be true?

        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
        // pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
        //                                     "y", 1, sensor_msgs::PointField::FLOAT32,
        //                                     "z", 1, sensor_msgs::PointField::FLOAT32,
        //                                     "rgb", 1, sensor_msgs::PointField::FLOAT32);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

        for (int i = 0; i < numVertices; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            *iter_x = mesh.vertices.at(i*3);
            *iter_y = mesh.vertices.at(i*3 + 1);
            *iter_z = mesh.vertices.at(i*3  + 2);
            //*iter_x = 5;
            //*iter_y = 0;
            //*iter_z = 0;
            *iter_r = *iter_g = *iter_b = 128;
        }

        mesh_vertex_pub_.publish(points_msg);
        return;
    }
    /*
    if (mesh.name == "person_actor" && mesh.ownerHasOwner) {
        PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
        int numVertices = mesh.vertices.size() / 3;
        //points_msg->header = disp_msg->header;
        points_msg->header.frame_id = "map";
        //ponts_msg->header.stamp = now();
        points_msg->height = 1;
        points_msg->width = numVertices;
        points_msg->is_bigendian = false;
        points_msg->is_dense = true; //or should be false?
        points_msg->point_step = 12;
        points_msg->row_step = 12*numVertices;

        points_msg->fields.resize(3);

        points_msg->fields[0].name = "x";
        points_msg->fields[0].offset = 0;
        points_msg->fields[0].datatype = 7;
        points_msg->fields[0].count = 1;

        points_msg->fields[1].name = "y";
        points_msg->fields[1].offset = 4;
        points_msg->fields[1].datatype = 7;
        points_msg->fields[1].count = 1;

        points_msg->fields[2].name = "z";
        points_msg->fields[2].offset = 8;
        points_msg->fields[2].datatype = 7;
        points_msg->fields[2].count = 1;

        points_msg->data.resize(12*numVertices);
        int index = 0;
        for (int i = 0; i < numVertices; i++) {
            uint_array = reinterpret_cast<uint8_t*>(&mesh.vertices.at(i*3));
            points_msg->data[index++] = (uint8_t) uint_array[0];
            points_msg->data[index++] = (uint8_t) uint_array[1];
            points_msg->data[index++] = (uint8_t) uint_array[2];
            points_msg->data[index++] = (uint8_t) uint_array[3];

            uint_array = reinterpret_cast<uint8_t*>(&mesh.vertices.at(i*3 + 1));
            points_msg->data[index++] = (uint8_t) uint_array[0];
            points_msg->data[index++] = (uint8_t) uint_array[1];
            points_msg->data[index++] = (uint8_t) uint_array[2];
            points_msg->data[index++] = (uint8_t) uint_array[3];

            uint_array = reinterpret_cast<uint8_t*>(&mesh.vertices.at(i*3 + 2));
            points_msg->data[index++] = (uint8_t) uint_array[0];
            points_msg->data[index++] = (uint8_t) uint_array[1];
            points_msg->data[index++] = (uint8_t) uint_array[2];
            points_msg->data[index++] = (uint8_t) uint_array[3];
        }


        mesh_vertex_pub_.publish(points_msg);
        return;
    }*/
    }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "mesh_info_node");
  ROS_INFO("mesh_info_node initialized");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  MeshInfoNode node;
//  node.testWithVideo();
  ros::spin();
  return 0;
}