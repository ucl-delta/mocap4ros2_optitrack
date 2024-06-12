// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap4r2_optitrack_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode("mocap4r2_optitrack_driver_node")
{
  declare_parameter<std::string>("connection_type", "Unicast");
  declare_parameter<std::string>("server_address", "000.000.000.000");
  declare_parameter<std::string>("local_address", "000.000.000.000");
  declare_parameter<std::string>("multicast_address", "000.000.000.000");
  declare_parameter<uint16_t>("server_command_port", 0);
  declare_parameter<uint16_t>("server_data_port", 0);

  declare_parameter<std::string>("parent_frame", "base");
  declare_parameter<bool>("enable_transform_broadcast", false);
  declare_parameter<bool>("enable_individual_pose_publisher", false);

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode()
{
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void
OptitrackDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void
OptitrackDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

std::chrono::nanoseconds OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData * data)
{
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable) {
    const double clientLatencySec =
      client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
    const double clientLatencyMillisec = clientLatencySec * 1000.0;
    const double transitLatencyMillisec =
      client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    const double largeLatencyThreshold = 100.0;
    if (clientLatencyMillisec >= largeLatencyThreshold) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500,
        "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
        largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
    }

    return round<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }
}

/**
 * \brief Get the latest active assets list from Motive.
 * \return 
 */
bool 
OptitrackDriverNode::update_data_description()
{
    // release memory allocated by previous in previous GetDataDescriptionList()
    if (data_descriptions)
    {
        NatNet_FreeDescriptions(data_descriptions);
    }

    // Retrieve Data Descriptions from Motive
    // printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
    int iResult = client->GetDataDescriptionList(&data_descriptions);
    if (iResult != ErrorCode_OK || data_descriptions == NULL)
    {
        return false;
    }

    return true;
}

std::string 
OptitrackDriverNode::get_rigidbody_name_from_id(int32_t id)
{
  sDataDescriptions* pDataDefs = data_descriptions;
  // printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
  for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
  {
      // printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
      if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
      {
          // RigidBody
          sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
          // printf("RigidBody Name : %s\n", pRB->szName);
          // printf("RigidBody ID : %d\n", pRB->ID);
          // printf("RigidBody Parent ID : %d\n", pRB->parentID);
          // printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
          if(pRB->ID == id) {
            // Return matching name
            return pRB->szName;
          }
      }
  }
  return "";
}

void
OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));

  // Check if list of models has changed
  bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
  if (bTrackedModelsChanged)
  {
      printf("\n\nMotive asset list changed.  Requesting new data descriptions.\n");
      if (update_data_description()) {
        // Probably put this call in a separate timer class 
        RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
      }
  }

  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;
  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = parent_frame_;
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap4r2_markers_pub_->publish(msg);
  }

  
  mocap4r2_msgs::msg::RigidBodies msg_rb;
  msg_rb.header.stamp = now() - frame_delay;
  msg_rb.header.frame_id = parent_frame_;
  msg_rb.frame_number = frame_number_;

  for (int i = 0; i < data->nRigidBodies; i++) {
    std::string rb_name;
    int32_t rb_id = data->RigidBodies[i].ID;
    auto it = rigid_body_id_name_map_.find(rb_id);
    if (it == rigid_body_id_name_map_.end()) {
        // Need to lookup in the data description and add the entry into the map then return
        rb_name = get_rigidbody_name_from_id(data->RigidBodies[i].ID);
        if(rb_name.length() == 0) {
          rb_name = std::to_string(rb_id);
          RCLCPP_INFO(this->get_logger(), "Key Not Found %d, using id as name instead: %s", rb_id, rb_name.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Key %d Found, using name: %s", rb_id, rb_name.c_str());
        }
        rigid_body_id_name_map_[rb_id] = rb_name;
    } else {
      rb_name = it->second;
    }

    if(enable_transform_broadcast_) {
      geometry_msgs::msg::TransformStamped t;
      // Read message content and assign it to corresponding tf variables
      t.header.stamp = now() - frame_delay;
      t.header.frame_id = parent_frame_;
      t.child_frame_id = rb_name;
      t.transform.translation.x = data->RigidBodies[i].x;
      t.transform.translation.y = data->RigidBodies[i].y;
      t.transform.translation.z = data->RigidBodies[i].z;
      t.transform.rotation.x = data->RigidBodies[i].qx;
      t.transform.rotation.y = data->RigidBodies[i].qy;
      t.transform.rotation.z = data->RigidBodies[i].qz;
      t.transform.rotation.w = data->RigidBodies[i].qw;
      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    mocap4r2_msgs::msg::RigidBody rb;
    rb.rigid_body_name = rb_name;
    rb.pose.position.x = data->RigidBodies[i].x;
    rb.pose.position.y = data->RigidBodies[i].y;
    rb.pose.position.z = data->RigidBodies[i].z;
    rb.pose.orientation.x = data->RigidBodies[i].qx;
    rb.pose.orientation.y = data->RigidBodies[i].qy;
    rb.pose.orientation.z = data->RigidBodies[i].qz;
    rb.pose.orientation.w = data->RigidBodies[i].qw;
    rb.markers = marker2rb[data->RigidBodies[i].ID];
    msg_rb.rigidbodies.push_back(rb);


    if(enable_individual_pose_publisher_) {
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher; 
      auto it = rigid_body_publisher_map_.find(rb_name);
      if (it == rigid_body_publisher_map_.end()) {
          // Not found create publisher for this rigid_body
          publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
            "rigid_bodies/" + rb_name, rclcpp::QoS(1000) 
          );
          rigid_body_publisher_map_[rb_name] = publisher;
          
      } else {
        publisher = it->second;
      }
      
      geometry_msgs::msg::PoseStamped t;
      t.header.stamp = now() - frame_delay;
      t.header.frame_id = rb_name;
      t.pose.position.x = data->RigidBodies[i].x;
      t.pose.position.y = data->RigidBodies[i].y;
      t.pose.position.z = data->RigidBodies[i].z;
      t.pose.orientation.x = data->RigidBodies[i].qx;
      t.pose.orientation.y = data->RigidBodies[i].qy;
      t.pose.orientation.z = data->RigidBodies[i].qz;
      t.pose.orientation.w = data->RigidBodies[i].qw;
      publisher->publish(t);
    }
  }

  if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
    mocap4r2_rigid_body_pub_->publish(msg_rb);
  }
}

void 
OptitrackDriverNode::reset_rigid_body_map_cb(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void) msg;
  rigid_body_id_name_map_.clear();
  RCLCPP_INFO(this->get_logger(), "Internal Rigid Body Id Name Map Cleared");
  rigid_body_publisher_map_.clear();
  RCLCPP_INFO(this->get_logger(), "Internal Rigid Body Name Pose Publishing Map Cleared");
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));
  
  if(enable_transform_broadcast_ || enable_individual_pose_publisher_)
  {
    mocap4r2_reset_rigid_body_list_pub_ = create_subscription<std_msgs::msg::Empty>(
        "reset_rigid_bodies", rclcpp::QoS(10), 
        std::bind(&OptitrackDriverNode::reset_rigid_body_map_cb, this, std::placeholders::_1));
  }

  if(enable_transform_broadcast_) {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  connect_optitrack();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  // release memory allocated by previous in previous GetDataDescriptionList()
  if (data_descriptions)
  {
      NatNet_FreeDescriptions(data_descriptions);
  }

  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();

  return ControlledLifecycleNode::on_error(state);
}

bool
OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description, 0, sizeof(server_description));
    client->GetServerDescription(&server_description);
    if (!server_description.HostPresent) {
      RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
      RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(
      get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
      server_description.szHostApp, server_description.HostAppVersion[0],
      server_description.HostAppVersion[1], server_description.HostAppVersion[2],
      server_description.HostAppVersion[3]);
    RCLCPP_INFO(
      get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
      server_description.NatNetVersion[1],
      server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

    void * pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool
OptitrackDriverNode::disconnect_optitrack()
{
  void * response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void
OptitrackDriverNode::initParameters()
{
  get_parameter<std::string>("connection_type", connection_type_);
  get_parameter<std::string>("server_address", server_address_);
  get_parameter<std::string>("local_address", local_address_);
  get_parameter<std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);

  get_parameter<std::string>("parent_frame", parent_frame_);
  get_parameter<bool>("enable_transform_broadcast", enable_transform_broadcast_);
  get_parameter<bool>("enable_individual_pose_publisher", enable_individual_pose_publisher_);
}

}  // namespace mocap4r2_optitrack_driver
