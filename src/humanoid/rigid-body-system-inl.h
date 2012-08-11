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

  inline SupportState &currentSupport() {
    return currentSupport_;
  };
  inline const SupportState &currentSupport() const {
    return currentSupport_;
  };
  inline void currentSupport(const SupportState &currentSupport) {
    currentSupport_ = currentSupport;
  };
  inline RobotData &data_robot() {
    return data_robot_;
  };
  inline RigidBody *com() {
    return com_;
  };
  inline const RigidBody *com() const {
    return com_;
  };
  inline RigidBody *left_foot() {
    return foot_left_;
  };
  inline const RigidBody *left_foot() const {
    return foot_left_;
  };
  inline RigidBody *right_foot() {
    return foot_right_;
  };
  inline const RigidBody *right_foot() const {
    return foot_right_;
  };



#endif // RIGID_BODY_SYSTEM_INL_H
