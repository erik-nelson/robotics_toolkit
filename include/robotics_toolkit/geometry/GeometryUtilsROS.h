/*
  geometry_utils: Utility library to provide common geometry types and transformations

  Copyright (C) 2015  Erik Nelson
  Copyright (C) 2013  Nathan Michael

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef __ROBOTICS_TOOLKIT_GEOMETRY_UTILS_ROS_H__
#define __ROBOTICS_TOOLKIT_GEOMETRY_UTILS_ROS_H__

#include <robotics_toolkit/geometry/GeometryUtils.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

namespace robotics_toolkit
{
  namespace geometry
  {
    namespace ros
    {

      namespace la = ::robotics_toolkit::linear_algebra;

      inline la::Vec3 fromROS(const geometry_msgs::Point& p)
      {
        return la::Vec3(p.x, p.y, p.z);
      }

      inline la::Vec3 fromROS(const geometry_msgs::Point32& p)
      {
        return la::Vec3(p.x, p.y, p.z);
      }

      inline la::Vec3 fromROS(const geometry_msgs::Vector3& p)
      {
        return la::Vec3(p.x, p.y, p.z);
      }

      inline Quat fromROS(const geometry_msgs::Quaternion& msg)
      {
        return Quat(msg.w, msg.x, msg.y, msg.z);
      }

      inline Transform3 fromROS(const geometry_msgs::Pose& msg)
      {
        return Transform3(fromROS(msg.position),
                          quatToR(fromROS(msg.orientation)));
      }

      inline Transform3 fromROS(const geometry_msgs::Transform& msg)
      {
        return Transform3(fromROS(msg.translation),
                          quatToR(fromROS(msg.rotation)));
      }

      inline geometry_msgs::Point toPoint(const la::Vec2& v)
      {
        geometry_msgs::Point msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = 0.0;

        return msg;
      }

      inline geometry_msgs::Point toPoint(const la::Vec3& v)
      {
        geometry_msgs::Point msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = v(2);

        return msg;
      }

      inline geometry_msgs::Point32 toPoint32(const la::Vec2& v)
      {
        geometry_msgs::Point32 msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = 0.0f;

        return msg;
      }

      inline geometry_msgs::Point32 toPoint32(const la::Vec3& v)
      {
        geometry_msgs::Point32 msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = v(2);

        return msg;
      }

      inline geometry_msgs::Vector3 toVector3(const la::Vec2& v)
      {
        geometry_msgs::Vector3 msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = 0.0;

        return msg;
      }

      inline geometry_msgs::Vector3 toVector3(const la::Vec3& v)
      {
        geometry_msgs::Vector3 msg;
        msg.x = v(0);
        msg.y = v(1);
        msg.z = v(2);

        return msg;
      }

      inline geometry_msgs::Quaternion toQuatMsg(const Quat& quat)
      {
        geometry_msgs::Quaternion msg;
        msg.w = quat.w();
        msg.x = quat.x();
        msg.y = quat.y();
        msg.z = quat.z();

        return msg;
      }

      inline geometry_msgs::Pose toPose(const Transform2& trans)
      {
        geometry_msgs::Pose msg;
        msg.position = toPoint(trans.translation);
        msg.orientation = toQuatMsg(RToQuat(Rot3(trans.rotation)));

        return msg;
      }

      inline geometry_msgs::Pose toPose(const Transform3& trans)
      {
        geometry_msgs::Pose msg;
        msg.position = toPoint(trans.translation);
        msg.orientation = toQuatMsg(RToQuat(trans.rotation));

        return msg;
      }

      inline geometry_msgs::Transform toTransform(const Transform2& trans)
      {
        geometry_msgs::Transform msg;
        msg.translation = toVector3(trans.translation);
        msg.rotation = toQuatMsg(RToQuat(Rot3(trans.rotation)));

        return msg;
      }

      inline geometry_msgs::Transform toTransform(const Transform3& trans)
      {
        geometry_msgs::Transform msg;
        msg.translation = toVector3(trans.translation);
        msg.rotation = toQuatMsg(RToQuat(trans.rotation));

        return msg;
      }

      inline geometry_msgs::Quaternion ZYXToQuatMsg(const la::Vec3& angles)
      {
        return toQuatMsg(RToQuat(ZYXToR(angles)));
      }

      inline la::Vec3 quatMsgToZYX(const geometry_msgs::Quaternion& msg)
      {
        return RToZYX(quatToR(fromROS(msg)));
      }

      inline double getRoll(const geometry_msgs::Quaternion& q)
      {
        return Rot3(fromROS(q)).roll();
      }

      inline double getPitch(const geometry_msgs::Quaternion& q)
      {
        return Rot3(fromROS(q)).pitch();
      }

      inline double getYaw(const geometry_msgs::Quaternion& q)
      {
        return Rot3(fromROS(q)).yaw();
      }

    } //\namespace ros
  } //\namespace geometry
} //\namespace robotics_toolkit

#endif
