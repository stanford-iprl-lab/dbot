/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file pose_velocity_vector.h
 * \date August 2015
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Dense>
#include <dbot/pose/pose_vector.h>
#include <dbot/pose/euler_vector.h>
namespace dbot
{
/// basic functionality for both vectors and blocks ****************************
template <typename Base>
class PoseVelocityBase
{
public:
    static constexpr int POSE_INDEX = 0;
    static constexpr int LINEAR_VELOCITY_INDEX = 6;
    static constexpr int ANGULAR_VELOCITY_INDEX = 9;

    static constexpr int POSE_SIZE = 6;
    static constexpr int VELOCITY_SIZE = 3;

    static constexpr int SizeAtCompileTime = POSE_SIZE + 2 * VELOCITY_SIZE;

    // types *******************************************************************
    typedef Eigen::Matrix<Real, VELOCITY_SIZE, 1> VelocityVector;
    typedef Eigen::Ref<Eigen::Vector3d> VelocityBlock;

    // constructor and destructor **********************************************
    PoseVelocityBase() : vector_(Base::Zero()) {}
    PoseVelocityBase(const Base& vector) : vector_(vector) {}
    virtual ~PoseVelocityBase() noexcept {}
    // operators ***************************************************************
    template <typename T>
    void operator=(const Eigen::MatrixBase<T>& vector)
    {
        vector_ = vector;
    }

    // accessors ***************************************************************
    virtual PoseVector pose() const
    {
        return PoseVector(vector_.template segment<POSE_SIZE>(POSE_INDEX));
    }
    virtual PoseVector::HomogeneousMatrix homogeneous() const
    {
        return pose().homogeneous();
    }
    virtual PoseVector::Affine affine() const { return pose().affine(); }
    virtual PoseVector::Vector position() const { return pose().position(); }
    virtual EulerVector orientation() const { return pose().orientation(); }
    virtual VelocityVector linear_velocity() const
    {
        return VelocityVector(vector_.template segment<VELOCITY_SIZE>(LINEAR_VELOCITY_INDEX));
    }
    virtual VelocityVector angular_velocity() const
    {
        return VelocityVector(vector_.template segment<VELOCITY_SIZE>(ANGULAR_VELOCITY_INDEX));
    }
    virtual const Base& vector() const { return vector_; }

    // mutators ****************************************************************
    PoseBlock pose() { return PoseBlock(vector_.template segment<POSE_SIZE>(POSE_INDEX)); }
    virtual void homogeneous(
        const typename PoseBlock::HomogeneousMatrix& H)
    {
        pose().homogeneous(H);
    }
    virtual void affine(const PoseBlock::Affine& A)
    {
        pose().affine(A);
    }
    virtual void set_zero()
    {
        pose().setZero();
        set_zero_velocity();
    }
    virtual void set_zero_pose()
    {
        pose().setZero();
    }
    virtual void set_zero_velocity()
    {
        linear_velocity() = Eigen::Vector3d::Zero();
        angular_velocity() = Eigen::Vector3d::Zero();
    }

    PoseBlock::PositionBlock position()
    {
        return pose().position();
    }
    PoseBlock::OrientationBlock orientation()
    {
        return pose().orientation();
    }
    VelocityBlock linear_velocity()
    {
        return VelocityBlock(vector_.template segment<VELOCITY_SIZE>(LINEAR_VELOCITY_INDEX));
    }
    VelocityBlock angular_velocity()
    {
        return VelocityBlock(vector_.template segment<VELOCITY_SIZE>(ANGULAR_VELOCITY_INDEX));
    }

    template <typename PoseType>
    void apply_delta(const PoseType& delta_pose)
    {
        pose().apply_delta(delta_pose);
        linear_velocity() = delta_pose.linear_velocity();
        angular_velocity() = delta_pose.angular_velocity();
    }

    template <typename PoseType>
    void subtract(const PoseType& mean)
    {
        pose().subtract(mean);
    }

    typename Base::PlainObject operator*(double coeff) const {
      return coeff * vector_;
    }

    template<typename OtherBase>
    friend typename OtherBase::PlainObject operator*(double coeff, const PoseVelocityBase<OtherBase>& vector);

protected:
    Base vector_;
};


template<typename Base>
typename Base::PlainObject operator*(double coeff, const PoseVelocityBase<Base>& vector) {
  return coeff * vector.vector_;
}

/// implementation for vectors *************************************************
using PoseVelocityVector = PoseVelocityBase<Eigen::Matrix<Real,12,1>>;

/// implementation for blocks **************************************************
using PoseVelocityBlock = PoseVelocityBase<Eigen::Ref<Eigen::Matrix<Real,12,1>>>;
}
