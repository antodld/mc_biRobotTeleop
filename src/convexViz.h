#pragma once
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>
#include <SpaceVecAlg/SpaceVecAlg>

visualization_msgs::Marker initMarker(const std::string & frame_id, const std::string & name, size_t id, int32_t t)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = name;
  marker.id = static_cast<int>(id);
  marker.type = t;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(0.5);
  return marker;
}

void setMarkerPose(visualization_msgs::Marker & marker, const sva::PTransformd & pt)
{
  auto q = Eigen::Quaterniond(pt.rotation().transpose());
  const auto & t = pt.translation();
  marker.pose.orientation.w = q.w();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.position.x = t.x();
  marker.pose.position.y = t.y();
  marker.pose.position.z = t.z();
}

visualization_msgs::Marker fromCylinder(const std::string & frame_id,
                                        const std::string & name,
                                        size_t id,
                                        sch::S_Cylinder & cylinder,
                                        const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::CYLINDER);
  marker.scale.x = 2 * cylinder.getRadius();
  marker.scale.y = 2 * cylinder.getRadius();
  marker.scale.z = (cylinder.getP2() - cylinder.getP1()).norm();
  auto midP = cylinder.getP1() + (cylinder.getP2() - cylinder.getP1()) / 2;
  Eigen::Vector3d midPV3{midP.m_x, midP.m_y, midP.m_z};
  
  auto p1_sch = cylinder.getP1();
  Eigen::Vector3d p1{p1_sch.m_x, p1_sch.m_y, p1_sch.m_z};
  auto p2_sch = cylinder.getP2();
  Eigen::Vector3d p2{p2_sch.m_x, p2_sch.m_y, p2_sch.m_z}; 
  Eigen::Vector3d z = (p2 - p1).normalized();
  assert(z.norm() != 0);
  Eigen::Vector3d z_2;
  if( z.cross(Eigen::Vector3d{0,0,1}).norm() != 0 )
  {
    z_2 = Eigen::Vector3d{0,0,1};
  }
  else if(z.cross(Eigen::Vector3d{0,1,0}).norm() != 0)
  {
    z_2 = Eigen::Vector3d{0,1,0};
  }
  else
  {
    z_2 = Eigen::Vector3d{1,0,0};
  }
  Eigen::Vector3d y = (z.cross(z_2)).normalized();
  Eigen::Vector3d x = y.cross(z).normalized();
  Eigen::Matrix3d R_1_0;
  R_1_0.col(0) = x; R_1_0.col(1) = y; R_1_0.col(2) = z; 
  // std::cout << "z\n" << z << std::endl;
  // std::cout << "z_2\n" << z_2 << std::endl;
  // std::cout << "R\n" << R_1_0 << std::endl;
  
  sva::PTransformd cylinderCenter{R_1_0.transpose(), midPV3};
  setMarkerPose(marker, cylinderCenter * colTrans);
  return marker;
}

visualization_msgs::Marker fromPolyhedron(const std::string & frame_id,
                                          const std::string & name,
                                          size_t id,
                                          sch::S_Polyhedron & poly,
                                          const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::TRIANGLE_LIST);
  auto & pa = *poly.getPolyhedronAlgorithm();
  const auto triangles = pa.triangles_;
  const auto vertexes = pa.vertexes_;
  for(unsigned int i = 0; i < triangles.size(); i++)
  {
    const auto a = vertexes[triangles[i].a]->getCoordinates();
    const auto b = vertexes[triangles[i].b]->getCoordinates();
    const auto c = vertexes[triangles[i].c]->getCoordinates();
    sva::PTransformd va(Eigen::Vector3d(a[0], a[1], a[2]));
    sva::PTransformd vb(Eigen::Vector3d(b[0], b[1], b[2]));
    sva::PTransformd vc(Eigen::Vector3d(c[0], c[1], c[2]));
    const auto normal = triangles[i].normal;
    auto cross = (a - b) ^ (a - c);
    auto dot = normal * (cross / cross.norm());
    bool reversePointOrder = dot < 0;
    std::vector<sva::PTransformd> vertexOrder = {va, vb, vc};
    if(reversePointOrder)
    {
      vertexOrder = {vc, vb, va};
    }
    for(size_t j = 0; j < vertexOrder.size(); j++)
    {
      vertexOrder[j] = vertexOrder[j] * colTrans;
      geometry_msgs::Point p;
      p.x = vertexOrder[j].translation().x();
      p.y = vertexOrder[j].translation().y();
      p.z = vertexOrder[j].translation().z();
      marker.points.push_back(p);
    }
  }
  return marker;
}

sch::Matrix4x4 convertTransfo(const sva::PTransformd & X_0_cvx)
{
  sch::Matrix4x4 m;
  const Eigen::Matrix3d & rot = X_0_cvx.rotation();
  const Eigen::Vector3d & tran = X_0_cvx.translation();

  for(unsigned int i = 0; i < 3; ++i)
  {
    for(unsigned int j = 0; j < 3; ++j) { m(i, j) = rot(j, i); }
  }

  m(0, 3) = tran(0);
  m(1, 3) = tran(1);
  m(2, 3) = tran(2);

  return m;
}