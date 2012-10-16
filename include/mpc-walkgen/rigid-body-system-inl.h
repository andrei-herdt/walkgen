#pragma once
#ifndef RIGID_BODY_SYSTEM_INL_H
#define RIGID_BODY_SYSTEM_INL_H


public:
  inline SupportState &current_support() {
    return current_support_;
  };
  inline const SupportState &current_support() const {
    return current_support_;
  };
  inline void current_support(const SupportState &current_support) {
    current_support_ = current_support;
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
