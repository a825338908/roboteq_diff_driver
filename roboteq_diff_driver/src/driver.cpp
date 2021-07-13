#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>


#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))


//
// cmd_vel subscriber
//

// Define following to enable cmdvel debug output
//#define _CMDVEL_DEBUG

// Define following to enable motor test mode
//  Runs both motors in forward direction at 10% of configured maximum (rpms in close_loop mode, power in open_loop mode)
//  If configured correctly robot should translate forward in a straight line
//#define _CMDVEL_FORCE_RUN

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG
#define _ODOM_ENABLED


// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/OdometryCovariances.h"
#include "rogoteq_diff_msgs/RequestOdometryCovariances.h"
#endif



void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}


uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}

class MainNode
{

public:
  MainNode();

public:
  //
  // cmd_vel subscriber
  //
  void cmdvel_callback( const geometry_msgs::Twist& twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();

  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream();
  void odom_front_loop();
  void odom_rear_loop();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish();
#ifdef _ODOM_COVAR_SERVER
  void odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res);
#endif

  int run();

protected:
  ros::NodeHandle nh;

  serial::Serial controller_front;
  serial::Serial controller_rear;

  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  //
  // odom publisher
  //
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;

  // buffer for reading encoder counts
  int odom_idx_front;
  char odom_buf_front[24];


  int odom_idx_rear;
  char odom_buf_rear[24];

  char odom_encoder_toss_front;
  char odom_encoder_toss_rear;

  int32_t odom_encoder_front_left;
  int32_t odom_encoder_front_right;
  int32_t odom_encoder_rear_left;
  int32_t odom_encoder_rear_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port_front;
  std::string port_rear;
  int baud;
  double wheel_circumference;
  double wheels_x_distance;
  double wheels_y_distance;
  int encoder_ppr;
  int encoder_cpr;
  double speed_factor;
};

MainNode::MainNode() :
  starttime(0),
  hstimer(0),
  mstimer(0),
  odom_idx_front(0),
  odom_encoder_toss_front(5),
  odom_encoder_toss_rear(5),
  odom_encoder_front_left(0),
  odom_encoder_front_right(0),
  odom_encoder_rear_left(0),
  odom_encoder_rear_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),
  pub_odom_tf(true),
  baud(115200),
  wheel_circumference(0),
  wheels_x_distance(0),
  wheels_y_distance(0),
  encoder_ppr(0),
  encoder_cpr(0),
  speed_factor(0.0)
{


  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("port_front", port_front, "/dev/motor_f");
  ROS_INFO_STREAM("port_front: " << port_front);
  nhLocal.param<std::string>("port_rear", port_rear, "/dev/motor_r");
  ROS_INFO_STREAM("port_rear: " << port_rear);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);

  nhLocal.param("wheel_circumference", wheel_circumference, 0.5124);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("wheels_x_distance", wheels_x_distance, 0.50); // TODO  0.43  real_x 0.43 real_y 0.49
  ROS_INFO_STREAM("wheels_x_distance: " << wheels_x_distance);
  nhLocal.param("wheels_y_distance", wheels_y_distance, 0.53); // TODO  0.26
  ROS_INFO_STREAM("wheels_y_distance: " << wheels_y_distance);

  nhLocal.param("encoder_ppr", encoder_ppr, 500);
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("encoder_cpr", encoder_cpr, 2000);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
  nhLocal.param("speed_factor", speed_factor, 7.3);
  ROS_INFO_STREAM("speed_factor: " << speed_factor);


}


//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{
  std::stringstream front_right_cmd;
  std::stringstream front_left_cmd;

  std::stringstream rear_right_cmd;
  std::stringstream rear_left_cmd;
// --mecan
  float linear_x = twist_msg.linear.x;
  float linear_y = twist_msg.linear.y;
  float angular_z = twist_msg.angular.z;

  float linear_vel_x_mins;
  float linear_vel_y_mins;
  float angular_vel_z_mins;
  float tangential_vel;
  float x_rpm;
  float y_rpm;
  float tan_rpm;

  //convert m/s to m/min
  linear_vel_x_mins = linear_x * 60;
  linear_vel_y_mins = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins = angular_z * 60;

  tangential_vel = angular_vel_z_mins * ((wheels_x_distance / 2) + (wheels_y_distance / 2));
  //tangential_vel = angular_vel_z_mins * wheels_y_distance;
//  wheel_circumference = 0.471238898;
  x_rpm = linear_vel_x_mins / wheel_circumference;
  y_rpm = linear_vel_y_mins / wheel_circumference;
  tan_rpm = tangential_vel / wheel_circumference;

  int32_t front_left_rpm = x_rpm - y_rpm - tan_rpm; //rpm1
  int32_t front_right_rpm = x_rpm + y_rpm + tan_rpm;//rpm2
  int32_t rear_left_rpm = x_rpm + y_rpm - tan_rpm;//rpm3
  int32_t rear_right_rpm = x_rpm - y_rpm + tan_rpm;//rpm4

// --mecan

  #ifdef _CMDVEL_DEBUG
  ROS_INFO_STREAM("cmdvel linear x: " << linear_x << " y: " << linear_y);
  ROS_INFO_STREAM("linear_vel_x_mins: " << linear_vel_x_mins << " linear_vel_y_mins: " << linear_vel_y_mins);
  ROS_INFO_STREAM("cmdvel rpm front_right: " << front_right_rpm << " front_left: " << front_left_rpm);
  ROS_INFO_STREAM("cmdvel rpm rear_right: " << rear_right_rpm << " rear_left: " << rear_left_rpm);
  #endif


  front_right_cmd << "!S 2 " << front_right_rpm*speed_factor << "\r"; //7.3
  front_left_cmd << "!S 1 " << front_left_rpm*speed_factor<< "\r";

  rear_left_cmd << "!S 1 " << rear_left_rpm*speed_factor << "\r";
  rear_right_cmd << "!S 2 " << rear_right_rpm*speed_factor << "\r";



#ifndef _CMDVEL_FORCE_RUN
  controller_front.write(front_right_cmd.str());
  controller_front.write(front_left_cmd.str());

  controller_rear.write(rear_right_cmd.str());
  controller_rear.write(rear_left_cmd.str());

  controller_front.flush();
  controller_rear.flush();

#endif
}


void MainNode::cmdvel_setup()
{
  controller_front.write("!G 1 0\r");
  controller_front.write("!G 2 0\r");
  controller_front.write("!S 1 0\r");
  controller_front.write("!S 2 0\r");
  controller_front.flush();

  controller_rear.write("!G 1 0\r");
  controller_rear.write("!G 2 0\r");
  controller_rear.write("!S 1 0\r");
  controller_rear.write("!S 2 0\r");
  controller_rear.flush();

  // disable echo
  controller_front.write("^ECHOF 1\r");
  controller_front.flush();

  controller_rear.write("^ECHOF 1\r");
  controller_rear.flush();

  // enable watchdog timer (500 ms)
  //controller.write("^RWD 500\r");

  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1, &MainNode::cmdvel_callback, this);

}

void MainNode::cmdvel_loop()
{
}

void MainNode::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
/*
  std::stringstream right_cmd;
  std::stringstream left_cmd;
  right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
  left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
  controller.write(right_cmd.str());
  controller.write(left_cmd.str());
  controller.flush();
  */
#endif
}


//
// odom publisher
//

#ifdef _ODOM_COVAR_SERVER
void MainNode::odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
#endif


void MainNode::odom_setup()
{

  if ( pub_odom_tf )
  {
    ROS_INFO("Broadcasting odom tf");
//    odom_broadcaster.init(nh);	// ???
  }

  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 100);

#ifdef _ODOM_COVAR_SERVER
  ROS_INFO("Advertising service on roboteq/odom_covar_srv");
  odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &MainNode::odom_covar_callback, this);
#endif

  tf_msg.header.seq = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;

  odom_msg.pose.covariance.assign(0);
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;

  odom_msg.twist.covariance.assign(0);
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;


  // start encoder streaming
  odom_stream();
  odom_last_time = millis();
}

void MainNode::odom_stream()
{
  controller_front.write("# C_?CR_# 33\r");
  controller_front.flush();

  controller_rear.write("# C_?CR_# 33\r");
  controller_rear.flush();

}


void MainNode::odom_front_loop()
{

  uint32_t nowtime = millis();

  // if we haven't received encoder counts in some time then restart streaming
  if( DELTAT(nowtime,odom_last_time) >= 1000 )
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  // read sensor data stream from motor controller
  if (controller_front.available())
  {
    char ch = 0;
    if ( controller_front.read((uint8_t*)&ch, 1) == 0 )
      return;
    if (ch == '\r')
    {
      odom_buf_front[odom_idx_front] = 0;
      // CR= is encoder counts
      if ( odom_buf_front[0] == 'C' && odom_buf_front[1] == 'R' && odom_buf_front[2] == '=' )
      {
        int delim;
        for ( delim = 3; delim < odom_idx_front; delim++ )
        {
          if ( odom_encoder_toss_front > 0 )
          {
            --odom_encoder_toss_front;
            break;
          }
          if (odom_buf_front[delim] == ':')
          {
            odom_buf_front[delim] = 0;
            //odom_encoder_front_right = (int32_t)strtol(odom_buf_front+3, NULL, 10);
            //odom_encoder_front_left = ((int32_t)strtol(odom_buf_front+delim+1, NULL, 10));
            odom_encoder_front_right = ((int32_t)strtol(odom_buf_front+delim+1, NULL, 10));
            odom_encoder_front_left = (int32_t)strtol(odom_buf_front+3, NULL, 10);
            #ifdef _ODOM_DEBUG
            ROS_INFO_STREAM("encoder front right: " << odom_encoder_front_right << " left: " << odom_encoder_front_left);
            #endif

            break;
          }
        }
      }
      odom_idx_front = 0;
    }
    else if ( odom_idx_front < (sizeof(odom_buf_front)-1) )
    {
      odom_buf_front[odom_idx_front++] = ch;
    }
  }
}


void MainNode::odom_rear_loop()
{

  uint32_t nowtime = millis();

  // if we haven't received encoder counts in some time then restart streaming
  if( DELTAT(nowtime,odom_last_time) >= 1000 )
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  // read sensor data stream from motor controller
  if (controller_rear.available())
  {
    char ch = 0;
    if ( controller_rear.read((uint8_t*)&ch, 1) == 0 )
      return;
    if (ch == '\r')
    {
      odom_buf_rear[odom_idx_rear] = 0;
      // CR= is encoder counts
      if ( odom_buf_rear[0] == 'C' && odom_buf_rear[1] == 'R' && odom_buf_rear[2] == '=' )
      {
        int delim;
        for ( delim = 3; delim < odom_idx_rear; delim++ )
        {
          if ( odom_encoder_toss_rear > 0 )
          {
            --odom_encoder_toss_rear;
            break;
          }
          if (odom_buf_rear[delim] == ':')
          {
            odom_buf_rear[delim] = 0;
            odom_encoder_rear_left = ((int32_t)strtol(odom_buf_rear+3, NULL, 10));
            odom_encoder_rear_right = (int32_t)strtol(odom_buf_rear+delim+1, NULL, 10);
            #ifdef _ODOM_DEBUG
            ROS_INFO_STREAM("encoder rear right: " << odom_encoder_rear_right << " left: " << odom_encoder_rear_left);
            #endif
            odom_publish();
            break;
          }
        }
      }
      odom_idx_rear = 0;
    }
    else if ( odom_idx_rear < (sizeof(odom_buf_rear)-1) )
    {
      odom_buf_rear[odom_idx_rear++] = ch;
    }
  }
}

void MainNode::odom_ms_run()
{

}

void MainNode::odom_ls_run()
{

}

void MainNode::odom_publish()
{

  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
//  ROS_INFO_STREAM("dt: " << dt);
  float dtm = dt / 60.0;
//  ROS_INFO_STREAM("dtm: " << dtm);
  odom_last_time = nowtime;

//mecan

  float rpm1 = (float)odom_encoder_front_left*60/(float)encoder_cpr*-1;//( (float)odom_encoder_front_left / (float)encoder_cpr ) / dtm;
  float rpm2 = (float)odom_encoder_front_right*60/(float)encoder_cpr*-1;//( (float)odom_encoder_front_right / (float)encoder_cpr ) / dtm;
  float rpm3 = (float)odom_encoder_rear_left*60/(float)encoder_cpr;//( (float)odom_encoder_rear_left / (float)encoder_cpr ) / dtm;
  float rpm4 = (float)odom_encoder_rear_right*60/(float)encoder_cpr;//( (float)odom_encoder_rear_right / (float)encoder_cpr ) / dtm;

  rpm1 = rpm1*-1;
  rpm2 = rpm2*-1;

  #ifdef _ODOM_DEBUG
  ROS_INFO_STREAM("rpm1 front_left: " << rpm1);
  ROS_INFO_STREAM("rpm2 front_right: " << rpm2);
  ROS_INFO_STREAM("rpm3 rear_left: " << rpm3);
  ROS_INFO_STREAM("rpm4 rear_right: " << rpm4);
  #endif

  float average_rps_x;
  float average_rps_y;
  float average_rps_a;

  //convert average revolutions per minute to revolutions per second
  //wheel_circumference = 0.471238898;
  average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4)/4)/60; // RPM
  float linear_x = average_rps_x * wheel_circumference; // m/s
  #ifdef _ODOM_DEBUG
  ROS_INFO_STREAM("average_rps_x: " << average_rps_x);
  ROS_INFO_STREAM("linear_x: " << linear_x);
  #endif
  //convert average revolutions per minute in y axis to revolutions per second
//  average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / 4)/60; // RPM
  average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / 4)/60; // RPM
  float linear_y = average_rps_y * wheel_circumference; // m/s

  //convert average revolutions per minute to revolutions per second
//  average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / 4)/ 60;
  average_rps_a = (float)(-rpm1 + rpm2 - rpm3 + rpm4)/4/ 60;
  float angular =  (average_rps_a * wheel_circumference) / ((wheels_x_distance / 2) + (wheels_y_distance / 2)); //  rad/s
  #ifdef _ODOM_DEBUG
  ROS_INFO_STREAM("average_rps_a: " << average_rps_a);
  ROS_INFO_STREAM("angular: " << angular);
  //ROS_INFO_STREAM("wheel_x: " << wheels_x_distance <<"wheel_y: "<<wheels_y_distance);
  ROS_INFO_STREAM("odom_yaw before + angular: " << odom_yaw);
  ROS_INFO_STREAM("odom_yaw - angular*dt: " << odom_yaw);
  #endif


//mecan


  // Update odometry

  double delta_yaw = angular*dt*1.2;
  double delta_x = (linear_x * cos(odom_yaw) - linear_y * sin(odom_yaw)) * dt;
  double delta_y = (linear_x * sin(odom_yaw) + linear_y * cos(odom_yaw)) * dt;

  //current pos
  odom_x += delta_x;
  odom_y += delta_y;
  odom_yaw += delta_yaw;

  /*
  odom_y += (linear_x * sin(odom_yaw) + linear_y * cos(odom_yaw)) * dt;
  odom_x += (linear_x * cos(odom_yaw) - linear_y * sin(odom_yaw)) * dt;
  odom_yaw = odom_yaw + angular*dt;
  */

  // Calculate velocities
/*
  float vx = ((odom_x - odom_last_x) / dt);
  float vy = ((odom_y - odom_last_y) / dt);
  //else
  float vyaw = (odom_yaw - odom_last_yaw) / dt;
  ROS_INFO_STREAM("vyaw: " << vyaw <<"odom_yaw: " << odom_yaw <<"odom_last_yaw: " <<odom_last_yaw);
  ROS_INFO_STREAM("vx: " << vx <<"odom_x: " << odom_x <<"odom_last_x: " <<odom_last_x);
  ROS_INFO_STREAM("vy: " << vy <<"odom_y: " << odom_y <<"odom_last_y: " <<odom_last_y);

  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;
*/
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  if ( pub_odom_tf )
  {
    tf_msg.header.seq++;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.transform.translation.x = odom_x;
    tf_msg.transform.translation.y = odom_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_broadcaster.sendTransform(tf_msg);
  }

  odom_msg.header.seq++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
//  ROS_INFO_STREAM("position_X: " << odom_x);
//  ROS_INFO_STREAM("position_y: " << odom_y);
  odom_msg.pose.pose.orientation = quat;
//  ROS_INFO_STREAM("odom_yaw: " << odom_yaw);
//  ROS_INFO_STREAM("quat: " << quat);
  odom_msg.twist.twist.linear.x = linear_x;
  odom_msg.twist.twist.linear.y = linear_y;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = angular;
  odom_pub.publish(odom_msg);

}


int MainNode::run()
{

	ROS_INFO("Beginning setup...");

  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial::Timeout timeout2 = serial::Timeout::simpleTimeout(1000);
	controller_rear.setPort(port_rear);
	controller_rear.setBaudrate(baud);
	controller_rear.setTimeout(timeout);

  controller_front.setPort(port_front);
	controller_front.setBaudrate(baud);
	controller_front.setTimeout(timeout2);


	// TODO: support automatic re-opening of port after disconnection
	while ( ros::ok() )
	{
    ROS_INFO_STREAM("Opening serial port on " << port_front << " at " << baud << "..." );
    ROS_INFO_STREAM("Opening serial port on " << port_rear << " at " << baud << "..." );
		try
		{
			controller_front.open();
			if ( controller_front.isOpen() )
			{
				ROS_INFO("Successfully opened serial controller_front");
  			controller_rear.open();
  			if ( controller_rear.isOpen() )
  			{
  				ROS_INFO("Successfully opened serial controller_rear");
  				break;
  			}
      }
		}
		catch (serial::IOException e)
		{
			ROS_WARN_STREAM("serial::IOException: " << e.what());
		}
		ROS_WARN("Failed to open serial port");
		sleep( 5 );
	}

	cmdvel_setup();
  #ifdef _ODOM_ENABLED
	odom_setup();
  #endif
  starttime = millis();
  hstimer = starttime;
  mstimer = starttime;
  lstimer = starttime;

  ros::Rate loop_rate(1000);

  ROS_INFO("Beginning looping...");

  while (ros::ok())
  {

    cmdvel_loop();
    #ifdef _ODOM_ENABLED
    odom_front_loop();
    odom_rear_loop();
    uint32_t nowtime = millis();
    #endif

//    // Handle 50 Hz publishing
//    if (DELTAT(nowtime,hstimer) >= 20)
    // Handle 30 Hz publishing
    if (DELTAT(nowtime,hstimer) >= 33)
    {
      hstimer = nowtime;
    }

    // Handle 10 Hz publishing
    if (DELTAT(nowtime,mstimer) >= 100)
    {
      mstimer = nowtime;
      cmdvel_run();
      #ifdef _ODOM_ENABLED
      odom_ms_run();
      #endif
    }

    // Handle 1 Hz publishing
    if (DELTAT(nowtime,lstimer) >= 1000)
    {
      lstimer = nowtime;
      #ifdef _ODOM_ENABLED
      odom_ls_run();
      #endif
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if ( controller_front.isOpen() )
    controller_front.close();

  if ( controller_rear.isOpen() )
    controller_rear.close();
  ROS_INFO("Exiting");

  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");

  MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}
