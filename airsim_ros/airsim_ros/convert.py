#!/usr/bin/env python3
from geometry_msgs.msg import Vector3, Quaternion
from airsim_ros_interfaces.msg import MultirotorState, RCData, KinematicsState, GeoPoint, CollisionInfo
from airsim.types import (MultirotorState as AsMultirotorState, RCData as AsRCData, KinematicsState as AsKinematicsState,
    GeoPoint as AsGeoPoint, CollisionInfo as AsCollisionInfo, Vector3r, Quaternionr)


def ros_from_vector3r(vector3r: Vector3r) -> Vector3:
    return Vector3(x=vector3r.x_val, y=vector3r.y_val, z=vector3r.z_val)


def ros_from_quaternionr(quaternionr: Quaternionr) -> Quaternion:
    return Quaternion(x=quaternionr.x_val, y=quaternionr.y_val, z=quaternionr.z_val, w=quaternionr.w_val)


def ros_from_collision_info(ci: AsCollisionInfo) -> CollisionInfo:
    msg = CollisionInfo()
    msg.normal = ros_from_vector3r(ci.normal)
    msg.impact_point = ros_from_vector3r(ci.impact_point)
    msg.position = ros_from_vector3r(ci.position)
    msg.penetration_depth = ci.penetration_depth
    msg.time_stamp = float(ci.time_stamp)
    msg.object_name = ci.object_name
    msg.object_id = ci.object_id
    return msg


def ros_from_kinematics_state(ks: AsKinematicsState) -> KinematicsState:
    msg = KinematicsState()
    msg.position = ros_from_vector3r(ks.position)
    msg.orientation = ros_from_quaternionr(ks.orientation)
    msg.linear_velocity = ros_from_vector3r(ks.linear_velocity)
    msg.angular_velocity = ros_from_vector3r(ks.angular_velocity)
    msg.linear_acceleration = ros_from_vector3r(ks.linear_acceleration)
    msg.angular_acceleration = ros_from_vector3r(ks.angular_acceleration)
    return msg


def ros_from_geo_point(v: AsGeoPoint) -> GeoPoint:
    msg = GeoPoint()
    msg.latitude = v.latitude
    msg.longitude = v.longitude
    msg.altitude = v.altitude
    return msg


def ros_from_rc_data(v: AsRCData) -> RCData:
    msg = RCData()
    msg.timestamp = v.timestamp
    msg.pitch = v.pitch
    msg.roll = v.roll
    msg.throttle = v.throttle
    msg.yaw = v.yaw
    msg.switch1 = v.switch1
    msg.switch2 = v.switch2
    msg.switch3 = v.switch3
    msg.switch4 = v.switch4
    msg.switch5 = v.switch5
    msg.switch6 = v.switch6
    msg.switch7 = v.switch7
    msg.switch8 = v.switch8
    msg.is_initialized = v.is_initialized
    msg.is_valid = v.is_valid
    return msg


def ros_from_multirotor_state(v: AsMultirotorState) -> MultirotorState:
    msg = MultirotorState()
    msg.collision = ros_from_collision_info(v.collision)
    msg.kinematics_estimated = ros_from_kinematics_state(v.kinematics_estimated)
    msg.gps_location = ros_from_geo_point(v.gps_location)
    msg.timestamp = v.timestamp
    msg.landed_state = v.landed_state
    msg.rc_data = ros_from_rc_data(v.rc_data)
    msg.ready = v.ready
    msg.ready_message = v.ready_message
    msg.can_arm = v.can_arm
    return msg
