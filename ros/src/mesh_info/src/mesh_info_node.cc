#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

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
#include<string>
#include <chrono>
//

//using namespace msr::airlib;
using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace geometry_msgs;

//
#define left_eye 0
#define right_eye 1
#define left_shoulder 2
#define right_shoulder 3
#define left_elbow 4
#define right_elbow 5
#define left_wrist 6
#define right_wrist 7
#define left_hip 8
#define right_hip 9
#define left_knee 10
#define right_knee 11
#define left_ankle 12
#define right_ankle 13 

class MeshInfoNode {
 public:
  MeshInfoNode();
  ~MeshInfoNode();

 private:
  void getMeshVertices(const ros::TimerEvent &e);
  MarkerArray getMarkerArray(std::vector<float> &bone_positions);
  Marker getSphereListMarker(std::vector<float> &bone_positions, float id);
  Marker getLineListMarker(std::vector<float> &bone_positions, float id);
  void getLine(Marker &marker, float a_x, float a_y, float a_z, 
                          float b_x, float b_y, float b_z);

  ros::Publisher mesh_vertex_pub_;
  ros::Publisher bone_pub_;
  ros::Timer publishing_timer_;
  double publish_freq_;
  std::string frame_id;
  std::string ns = "mesh_ns"; 
  std::string actor_name = "person_actor";
};

MeshInfoNode::MeshInfoNode() {
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  if (!priv_nh.getParam("publish_freq", publish_freq_)) { publish_freq_ = 10.0; }
  priv_nh.param<std::string>("frame_id", frame_id, "/world");

  mesh_vertex_pub_ = nh.advertise<sensor_msgs::PointCloud2>("mesh_vertices", 1);
  bone_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bone_markers", 1);
  publishing_timer_ = nh.createTimer(ros::Duration(1 / publish_freq_ ), &MeshInfoNode::getMeshVertices, this);
}

MeshInfoNode::~MeshInfoNode() {
}

void MeshInfoNode::getMeshVertices(const ros::TimerEvent &e) {
    msr::airlib::MultirotorRpcLibClient client;
    //client.confirmConnection();

    auto meshes = client.simGetSkeletalMeshPositionVertexBuffers(actor_name, true);
    
    uint8_t *uint_array;
    for(auto &mesh : meshes) {
      PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
      int numVertices = mesh.vertices.size() / 3;
      points_msg->header.frame_id = frame_id.c_str();
      points_msg->height = 1;
      points_msg->width = numVertices;
      points_msg->is_bigendian = false;
      //points_msg->is_dense = true; //or should be false?
      points_msg->is_dense = false; //or should be true?

      sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
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
          *iter_r = *iter_g = *iter_b = 128;
      }

      mesh_vertex_pub_.publish(points_msg);
      bone_pub_.publish(getMarkerArray(mesh.bone_positions));
      return;
  }
}

MarkerArray MeshInfoNode::getMarkerArray(std::vector<float> &bone_positions) {
  MarkerArray markerArray;
  //markerArray.markers[0] = getSphereListMarker(bone_positions, 0);
  //markerArray.markers[0] = getLineListMarker(bone_positions, 1);
  markerArray.markers.push_back(getSphereListMarker(bone_positions, 0));
  markerArray.markers.push_back(getLineListMarker(bone_positions, 1));
  return markerArray;
}

Marker MeshInfoNode::getSphereListMarker(std::vector<float> &bone_positions, float id) {
  Marker sphereListMarker;
  sphereListMarker.header.frame_id = frame_id.c_str();
  sphereListMarker.ns = ns.c_str();
  sphereListMarker.id = id;
  sphereListMarker.type = Marker::SPHERE_LIST;
  sphereListMarker.action = Marker::ADD;
  sphereListMarker.pose.position.x = 0.0;
  sphereListMarker.pose.position.y = 0.0;
  sphereListMarker.pose.position.z = 0.0;
  sphereListMarker.pose.orientation.w = 1.0;
  sphereListMarker.pose.orientation.x = 0.0;
  sphereListMarker.pose.orientation.y = 0.0;
  sphereListMarker.pose.orientation.z = 0.0;
  sphereListMarker.scale.x = 0.1;
  sphereListMarker.scale.y = 0.1;
  sphereListMarker.scale.z = 0.1;
  sphereListMarker.color.a = 1.0;
  sphereListMarker.color.r = 1.0;
  sphereListMarker.color.g = 0.0;
  sphereListMarker.color.b = 1.0;

  for (int i = 0; i < bone_positions.size()/3; i++) {
    Point point;
    point.x = bone_positions[3*i];
    point.y = bone_positions[3*i + 1];
    point.z = bone_positions[3*i + 2];
    sphereListMarker.points.push_back(point);
  }

  return sphereListMarker;
}

Marker MeshInfoNode::getLineListMarker(std::vector<float> &bone_positions, float id) {
  Marker lineListMarker;
  lineListMarker.header.frame_id = frame_id.c_str();
  lineListMarker.ns = ns.c_str();
  lineListMarker.id = id;
  lineListMarker.type = Marker::LINE_LIST;
  lineListMarker.action = Marker::ADD;
  lineListMarker.pose.position.x = 0.0;
  lineListMarker.pose.position.y = 0.0;
  lineListMarker.pose.position.z = 0.0;
  lineListMarker.pose.orientation.w = 1.0;
  lineListMarker.pose.orientation.x = 0.0;
  lineListMarker.pose.orientation.y = 0.0;
  lineListMarker.pose.orientation.z = 0.0;
  lineListMarker.scale.x = 0.1;
  lineListMarker.scale.y = 0.1;
  lineListMarker.scale.z = 0.1;
  lineListMarker.color.a = 1.0;
  lineListMarker.color.r = 1.0;
  lineListMarker.color.g = 1.0;
  lineListMarker.color.b = 0.0;

  getLine(lineListMarker, bone_positions[3*left_shoulder], bone_positions[3*left_shoulder + 1],
          bone_positions[3*left_shoulder + 2], bone_positions[3*right_shoulder], 
          bone_positions[3*right_shoulder + 1], bone_positions[3*right_shoulder + 2]);

  getLine(lineListMarker, bone_positions[3*left_shoulder], bone_positions[3*left_shoulder + 1],
          bone_positions[3*left_shoulder + 2], bone_positions[3*left_elbow], 
          bone_positions[3*left_elbow + 1], bone_positions[3*left_elbow + 2]);

  getLine(lineListMarker, bone_positions[3*right_shoulder], bone_positions[3*right_shoulder + 1],
          bone_positions[3*right_shoulder + 2], bone_positions[3*right_elbow], 
          bone_positions[3*right_elbow + 1], bone_positions[3*right_elbow + 2]);
  
  getLine(lineListMarker, bone_positions[3*left_elbow], bone_positions[3*left_elbow + 1],
          bone_positions[3*left_elbow + 2], bone_positions[3*left_wrist], 
          bone_positions[3*left_wrist + 1], bone_positions[3*left_wrist + 2]);
  
  getLine(lineListMarker, bone_positions[3*right_elbow], bone_positions[3*right_elbow + 1],
          bone_positions[3*right_elbow + 2], bone_positions[3*right_wrist], 
          bone_positions[3*right_wrist + 1], bone_positions[3*right_wrist + 2]);

  getLine(lineListMarker, bone_positions[3*left_hip], bone_positions[3*left_hip + 1],
          bone_positions[3*left_hip + 2], bone_positions[3*left_knee], 
          bone_positions[3*left_knee + 1], bone_positions[3*left_knee + 2]);

  getLine(lineListMarker, bone_positions[3*right_hip], bone_positions[3*right_hip + 1],
          bone_positions[3*right_hip + 2], bone_positions[3*right_knee], 
          bone_positions[3*right_knee + 1], bone_positions[3*right_knee + 2]);

  getLine(lineListMarker, bone_positions[3*left_knee], bone_positions[3*left_knee + 1],
          bone_positions[3*left_knee + 2], bone_positions[3*left_ankle], 
          bone_positions[3*left_ankle + 1], bone_positions[3*left_ankle + 2]);

  getLine(lineListMarker, bone_positions[3*right_knee], bone_positions[3*right_knee + 1],
          bone_positions[3*right_knee + 2], bone_positions[3*right_ankle], 
          bone_positions[3*right_ankle + 1], bone_positions[3*right_ankle + 2]);

  float mind_shoulder_x = (bone_positions[3*left_shoulder] + bone_positions[3*right_shoulder])/2;
  float mind_shoulder_y = (bone_positions[3*left_shoulder + 1] + bone_positions[3*right_shoulder + 1])/2;
  float mind_shoulder_z = (bone_positions[3*left_shoulder + 2] + bone_positions[3*right_shoulder + 2])/2;
  
  getLine(lineListMarker, mind_shoulder_x, mind_shoulder_y,
          mind_shoulder_z, bone_positions[3*left_hip], 
          bone_positions[3*left_hip + 1], bone_positions[3*left_hip + 2]);

  getLine(lineListMarker, mind_shoulder_x, mind_shoulder_y,
          mind_shoulder_z, bone_positions[3*right_hip], 
          bone_positions[3*right_hip + 1], bone_positions[3*right_hip + 2]);

  return lineListMarker;
}

void MeshInfoNode::getLine(Marker &marker, float a_x, float a_y, float a_z, 
                          float b_x, float b_y, float b_z) {
  Point point_a;
  point_a.x = a_x;
  point_a.y = a_y;
  point_a.z = a_z;

  Point point_b;
  point_b.x = b_x;
  point_b.y = b_y;
  point_b.z = b_z;

  marker.points.push_back(point_a);
  marker.points.push_back(point_b);
}

/*
Marker MeshInfoNode::getSphereListMarker(std::vector<float> &bone_positions, float id) {
  Marker sphereListMarker;
  sphereListMarker.header.frame_id = frame_id.c_str();
  sphereListMarker.ns = ns.c_str();
  sphereListMarker.id = id;
  sphereListMarker.type = Marker::SPHERE_LIST;
  sphereListMarker.action = Marker::ADD;
  sphereListMarker.pose.position.x = 0.0;
  sphereListMarker.pose.position.y = 0.0;
  sphereListMarker.pose.position.z = 0.0;
  sphereListMarker.pose.orientation.w = 1.0;
  sphereListMarker.pose.orientation.x = 0.0;
  sphereListMarker.pose.orientation.y = 0.0;
  sphereListMarker.pose.orientation.z = 0.0;
  sphereListMarker.scale.x = 0.1;
  sphereListMarker.scale.y = 0.1;
  sphereListMarker.scale.z = 0.1;
  sphereListMarker.color.a = 1.0;
  sphereListMarker.color.r = 1.0;
  sphereListMarker.color.g = 0.0;
  sphereListMarker.color.b = 1.0;
  sphereListMarker.points.resize(bone_positions.size()/3);

  for (int i = 0; i < bone_positions.size()/3; i++) {
    sphereListMarker.points[i].x = bone_positions.at(i*3);
    sphereListMarker.points[i].x = bone_positions.at(i*3 + 1);
    sphereListMarker.points[i].x = bone_positions.at(i*3 + 2);
  }

  return sphereListMarker;
}

Marker MeshInfoNode::getLineListMarker(std::vector<float> &bone_positions, float id) {
  Marker lineListMarker;
  lineListMarker.header.frame_id = frame_id;
  lineListMarker.ns = ns;
  lineListMarker.id = id;
  lineListMarker.type = Marker::LINE_LIST;
  lineListMarker.action = Marker::ADD;
  lineListMarker.pose.position.x = 0.0;
  lineListMarker.pose.position.y = 0.0;
  lineListMarker.pose.position.z = 0.0;
  lineListMarker.pose.orientation.w = 0.0;
  lineListMarker.pose.orientation.x = 0.0;
  lineListMarker.pose.orientation.y = 0.0;
  lineListMarker.pose.orientation.z = 0.0;
  lineListMarker.scale.x = 0.1;
  lineListMarker.scale.y = 0.1;
  lineListMarker.scale.z = 0.1;
  lineListMarker.color.a = 1.0;
  lineListMarker.color.r = 1.0;
  lineListMarker.color.g = 1.0;
  lineListMarker.color.b = 0.0;
  lineListMarker.points.resize(2*10);
  std::cerr<<"Here"<<std::endl;
  getLine(lineListMarker, 0, bone_positions, left_shoulder, right_shoulder);
  std::cerr<<"Here2"<<std::endl;
  getLine(lineListMarker, 1, bone_positions, left_shoulder, left_elbow);
  getLine(lineListMarker, 2, bone_positions, right_shoulder, right_elbow);
  getLine(lineListMarker, 3, bone_positions, left_elbow, left_wrist);
  getLine(lineListMarker, 4, bone_positions, right_elbow, right_wrist);
  getLine(lineListMarker, 5, bone_positions, left_hip, left_knee);
  getLine(lineListMarker, 6, bone_positions, right_hip, right_knee);
  getLine(lineListMarker, 7, bone_positions, left_knee, left_ankle);
  getLine(lineListMarker, 8, bone_positions, right_knee, right_ankle);
  std::cerr<<"Here3"<<std::endl;
  float midShoulder_x = (bone_positions.at(3*left_shoulder) + bone_positions.at(3*right_shoulder))/2;
  float midShoulder_y = (bone_positions.at(3*left_shoulder + 1) + bone_positions.at(3*right_shoulder + 1))/2;
  float midShoulder_z = (bone_positions.at(3*left_shoulder + 2) + bone_positions.at(3*right_shoulder + 2))/2 ; 
  getLine(lineListMarker, 9, bone_positions, left_hip, midShoulder_x, midShoulder_y, midShoulder_z);
  std::cerr<<"Here4"<<std::endl;
  getLine(lineListMarker, 10, bone_positions, right_hip, midShoulder_x, midShoulder_y, midShoulder_z);
  std::cerr<<"Here5"<<std::endl;
  return lineListMarker;
}

void MeshInfoNode::getLine(Marker &marker, int marker_index, std::vector<float> &bone_positions, 
            int bone_index_a, int bone_index_b) {
  marker.points[2*marker_index].x = bone_positions.at(3*bone_index_a);
  marker.points[2*marker_index].y = bone_positions.at(3*bone_index_a + 1);
  marker.points[2*marker_index].z = bone_positions.at(3*bone_index_a + 2);

  marker.points[2*marker_index + 1].x = bone_positions.at(3*bone_index_b);
  marker.points[2*marker_index + 1].y = bone_positions.at(3*bone_index_b + 1);
  marker.points[2*marker_index + 1].z = bone_positions.at(3*bone_index_b + 2);
}

void MeshInfoNode::getLine(Marker &marker, int marker_index, std::vector<float> &bone_positions, 
            int bone_index_a, float b_x, float b_y, float b_z) {
  marker.points[2*marker_index].x = bone_positions.at(3*bone_index_a);
  marker.points[2*marker_index].y = bone_positions.at(3*bone_index_a + 1);
  marker.points[2*marker_index].z = bone_positions.at(3*bone_index_a + 2);

  marker.points[2*marker_index + 1].x = b_x;
  marker.points[2*marker_index + 1].y = b_y;
  marker.points[2*marker_index + 1].z = b_z;
}*/


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