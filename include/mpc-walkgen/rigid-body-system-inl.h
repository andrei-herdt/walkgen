#pragma once
#ifndef RIGID_BODY_SYSTEM_INL_H
#define RIGID_BODY_SYSTEM_INL_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body-system_inl.h
///\author Andrei Herdt
///
////////////////////////////////////////////////////////////////////////////////



public:
  inline ConvexHull convexHull(HullType type, const SupportState &prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const {
    ConvexHull hull;
    convexHull(hull, type, prwSupport, computeLinearSystem, rotateHull);
    return hull;
  }
  void convexHull(ConvexHull &hull, HullType type, const SupportState &prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const;//TODO: Change this

  RigidBody *body(BodyType type);// TODO:To be removed
  const RigidBody *body(BodyType type) const;

  inline SupportState &current_support() {
    return currentSupport_;
  };
  inline const SupportState &current_support() const {
    return currentSupport_;
  };
  inline void current_support(const SupportState &current_support) {
    currentSupport_ = current_support;
  };
  inline RobotData &robot_data() {
    return robot_data_;
  };
  inline RigidBody *com() {
    return com_;
  };
  inline const RigidBody *com() const {
    return com_;
  };
  inline RigidBody *left_foot() {
    return left_foot_;
  };
  inline const RigidBody *left_foot() const {
    return left_foot_;
  };
  inline RigidBody *right_foot() {
    return right_foot_;
  };
  inline const RigidBody *right_foot() const {
    return right_foot_;
  };



#endif // RIGID_BODY_SYSTEM_INL_H
