#include <vector>
#include <tf/tf.h>
#include <robmovil_msgs/LandmarkArray.h>
#include <sensor_msgs/PointCloud.h>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros (0.1 = 10cm)

robmovil_ekf::LandmarkDetector::LandmarkDetector(ros::NodeHandle& _n) :
    n(_n), transform_received(false)
{
  laser_sub = n.subscribe("/scan", 1, &LandmarkDetector::on_laser_scan, this);
  landmark_pub = n.advertise<robmovil_msgs::LandmarkArray>("/landmarks", 1);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("/landmarks_pointcloud", 1);

  listener = boost::make_shared<tf::TransformListener>();

  n.param("robot_frame", robot_frame, std::string("base_link"));
  n.param("publish_robot_frame", publish_robot_frame, std::string("base_link"));
  n.param("laser_frame", laser_frame, std::string("laser"));

  ROS_INFO_STREAM("publishing to frame " << publish_robot_frame);
}

void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::LaserScanConstPtr& msg)
{
  if(!update_laser_tf(msg->header.stamp)){
    ROS_WARN_STREAM(laser_frame << " -> " << robot_frame << " transform not yet received, not publishing landmarks");
    return;
  }

  /* convertir range,bearing a puntos cartesianos x,y,0 (ojo: descarta puntos en el "infinito") */
  std::vector<tf::Vector3> cartesian;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    float d = msg->ranges[i];
    if (d <= msg->range_min || d >= msg->range_max) continue; /* ignora puntos en el "infinito" */
    float a = msg->angle_min + msg->angle_increment * i;

    tf::Vector3 p;
    p.setX(d * cos(a));
    p.setY(d * sin(a));
    p.setZ(0);

    cartesian.push_back(p);
  }

  /* recorro los puntos "validos" y busco grupos a distancias similares */
  robmovil_msgs::LandmarkArray landmark_array;
  landmark_array.header.stamp = msg->header.stamp;
  landmark_array.header.frame_id = publish_robot_frame;
  
  std::vector<tf::Vector3> landmark_points;
  std::vector<tf::Vector3> centroids;
  
  for (int i = 0; i < cartesian.size(); i++)
  {
    /* comenzamos a acumular mediciones referentes a un nuevo landmark */
    if (landmark_points.empty()){
      landmark_points.push_back(cartesian[i]);
      continue;
    }
    
    /* distancia entre la medicion actual y la anterior */
    float dist = (cartesian[i] - landmark_points.back()).length();
    
    /* si la distancia es pequeña las mediciones se consideran provenientes del mismo landmark */
    if(dist < LANDMARK_DIAMETER && i != (cartesian.size() - 1)){
      landmark_points.push_back(cartesian[i]);
      continue;
    }
    
    /* se terminaron las mediciones provenientes al landmark que se venia detectando,
     * se calcula la pose del landmark como el centroide de las mediciones */
     
    //ROS_INFO_STREAM("landmark con " << landmark_points.size() << " puntos");
    
    tf::Vector3 centroid(0,0,0);

    /* calculo del centroide */
    for (int j = 0; j < landmark_points.size(); j++)
      centroid += landmark_points[j];
    centroid /= landmark_points.size();

    /* convierto el centroide a coordenadas del robot */
    centroid = laser_transform * centroid;
    //ROS_INFO_STREAM("landmark detectado (en coordenadas del robot): " << centroid.getX() << " " << centroid.getY() << " " << centroid.getZ());

    centroids.push_back(centroid);

    /* se pasa el centroide a coordenadas polares */
    robmovil_msgs::Landmark landmark;
    
    float r = sqrt(centroid.getX()*centroid.getX() + centroid.getY()*centroid.getY());
    landmark.bearing = atan2(centroid.getY(), centroid.getX());
    landmark.range = r;

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);
    //ROS_INFO_STREAM("landmark detectado (polares): " << i << ": " << landmark.range << " " << landmark.bearing);

    /* empiezo a procesar un nuevo landmark */
    landmark_points.clear();
  }

  if (!landmark_array.landmarks.empty()){
    landmark_pub.publish(landmark_array);
    publish_pointcloud(landmark_array.header, centroids);
  }
}

bool robmovil_ekf::LandmarkDetector::update_laser_tf(const ros::Time& required_time)
{
  if (!listener->waitForTransform(laser_frame, robot_frame, required_time, ros::Duration(1)))
    return false;
  else
  {
    listener->lookupTransform(laser_frame, robot_frame, ros::Time(0), laser_transform);
    return true;
  }
}

void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::Header& header, const std::vector<tf::Vector3>& landmark_positions)
{
  sensor_msgs::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub.publish(pointcloud);
}
