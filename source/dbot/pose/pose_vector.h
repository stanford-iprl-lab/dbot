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
 * \file pose_vector.h
 * \date August 2015
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Dense>

#include "euler_vector.h"

namespace dbot
{
/// basic functionality for both vectors and blocks ****************************
template <typename Base>
class PoseBase
{
public:
    static constexpr int BLOCK_SIZE = 3;
    static constexpr int POSITION_INDEX = 0;
    static constexpr int EULER_VECTOR_INDEX = 3;

    // types *******************************************************************
    typedef Eigen::Matrix<Real, 3, 1> Vector;
    typedef Eigen::Matrix<Real, 4, 4> HomogeneousMatrix;
    typedef typename Eigen::Transform<Real, 3, Eigen::Affine> Affine;

    typedef Eigen::Ref<Eigen::Vector3d> PositionBlock;
    typedef EulerBlock OrientationBlock;

    typedef PoseBase<Eigen::Matrix<Real, 6, 1>> PoseVector;

    // constructor and destructor **********************************************
    PoseBase() : vector_(Base::Zero()) {}
    PoseBase(const Base& vector) : vector_(vector) {}
    virtual ~PoseBase() noexcept {}
    // operators ***************************************************************
    template <typename T>
    void operator=(const Eigen::MatrixBase<T>& vector)
    {
        vector_ = vector;
    }

    // accessors ***************************************************************
    virtual Vector position() const
    {
        return vector_.template segment<BLOCK_SIZE>(POSITION_INDEX);
    }
    virtual EulerVector orientation() const
    {
        return Eigen::Vector3d(vector_.template segment<BLOCK_SIZE>(EULER_VECTOR_INDEX));
    }
    virtual HomogeneousMatrix homogeneous() const
    {
        HomogeneousMatrix H(HomogeneousMatrix::Identity());
        H.topLeftCorner(3, 3) = orientation().rotation_matrix();
        H.topRightCorner(3, 1) = position();

        return H;
    }
    virtual Affine affine() const
    {
        Affine A;
        A.linear() = orientation().rotation_matrix();
        A.translation() = position();

        return A;
    }
    virtual PoseVector inverse() const
    {
        PoseVector inv(Base::Zero());
        inv.homogeneous(this->homogeneous().inverse());
        return inv;
    }
    virtual const Base& vector() const { return vector_; }

    // mutators ****************************************************************
    PositionBlock position() { return vector_.template segment<BLOCK_SIZE>(POSITION_INDEX); }
    OrientationBlock orientation()
    {
        return OrientationBlock(vector_.template segment<BLOCK_SIZE>(EULER_VECTOR_INDEX));
    }
    virtual void homogeneous(const HomogeneousMatrix& H)
    {
        orientation().rotation_matrix(H.topLeftCorner(3, 3));
        position() = H.topRightCorner(3, 1);
    }
    virtual void affine(const Affine& A)
    {
        orientation().rotation_matrix(A.rotation());
        position() = A.translation();
    }
    virtual void setZero() { vector_.setZero(); }
    virtual void set_zero() { setZero(); }
    template <typename PoseType>
    void apply_delta(const PoseType& delta_pose)
    {
        position() = orientation().rotation_matrix() * delta_pose.position()
                + position();
        orientation() = orientation() * delta_pose.orientation();
    }

    /// \todo: these subtract and apply_delta functions are a bit confusing,
    /// they should be made clearer

    // note: we do not apply the inverse pose, but we treat position and
    // orientation separately
    template <typename PoseType>
    void subtract(const PoseType& mean)
    {
        position() = mean.orientation().inverse().rotation_matrix()
                * (position() - mean.position());
        orientation() = mean.orientation().inverse() * orientation();
    }

    // operators ***************************************************************
    template <typename T>
    PoseVector operator*(const PoseBase<T>& factor)
    {
        PoseVector product(Base::Zero());
        product.homogeneous(this->homogeneous() * factor.homogeneous());
        return product;
    }
    Real operator()(int i, int j) const {
      return vector_(i, j);
    }
    Real& operator()(int i, int j) {
      return vector_(i, j);
    }
protected:
    Base vector_;
};

/// implementation for vectors *************************************************
using PoseVector = PoseBase<Eigen::Matrix<Real,6,1>>;

/// implementation for blocks **************************************************
using PoseBlock = PoseBase<Eigen::Ref<Eigen::Matrix<Real,6,1>>>;
}
