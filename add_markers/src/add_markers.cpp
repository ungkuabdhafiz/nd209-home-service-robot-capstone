#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include "tf2_msgs/TFMessage.h"

using namespace std;

float pickup_x = 0.0;
float pickup_y = 4.0;
float dropoff_x = 8.0;
float dropoff_y = 0.0;
bool pickup = false;
bool dropoff = false;

void tf_callback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
  float x = msg->transforms[0].transform.translation.x;
  float y = msg->transforms[0].transform.translation.y;

  float distance_pickup = sqrt(pow(x - pickup_x, 2) + pow(y - pickup_y, 2));
  float distance_dropoff = sqrt(pow(x - dropoff_x, 2) + pow(y - dropoff_y, 2));

  if (distance_pickup < 0.3 && (pickup == false))
  {
    ROS_INFO("Robot reached pickup zone");
    pickup = true;
  }

  if (distance_dropoff < 0.3 && (dropoff == false))
  {
    ROS_INFO("Robot reached dropoff zone");
    dropoff = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber tf_sub = n.subscribe("/tf", 1000, tf_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    //INITIAL
    if ((pickup == false) && (dropoff == false))
    {
      marker.pose.position.x = pickup_x;
      marker.pose.position.y = pickup_y;
      marker.pose.orientation.w = 1.0;

      marker.action = visualization_msgs::Marker::ADD;

      marker_pub.publish(marker);
    }

    //AFTER PICKUP
    if ((pickup == true) && (dropoff == false))
    {
      ros::Duration(2.0).sleep();

      marker.action = visualization_msgs::Marker::DELETE;

      marker_pub.publish(marker);
    }

    //AFTER DROPOFF
    if (dropoff == true)
    {
      ros::Duration(2.0).sleep();

      marker.pose.position.x = dropoff_x;
      marker.pose.position.y = dropoff_y;
      marker.pose.orientation.w = 1.0;

      marker.action = visualization_msgs::Marker::ADD;

      marker_pub.publish(marker);
    }

    //marker_pub.publish(marker);

    ros::spinOnce();
    r.sleep();
  }
}
