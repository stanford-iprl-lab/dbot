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
 * \file orientation_transition.hpp
 * \date July 2015
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Dense>

namespace dbot
{
typedef double Real;

/// basic functionality for both vectors and blocks ****************************
template <typename Base>
class EulerBase
{
public:
    // types *******************************************************************
    typedef typename Base::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1> Axis;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector;
    typedef Eigen::AngleAxis<Scalar> AngleAxis;
    typedef Eigen::Quaternion<Scalar> Quaternion;
    typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;

    typedef EulerBase<Eigen::Matrix<Real, 3, 1>> EulerVector;

    // constructor and destructor **********************************************
    EulerBase() : vector_(Base::Zero()) {}
    EulerBase(const Base& vector) : vector_(vector) {}
    EulerBase(const EulerBase<Base>& other) : vector_(other.vector()) {}
    template<typename OtherBase>
    EulerBase(const EulerBase<OtherBase>& other) : vector_(other.vector()) {}
    virtual ~EulerBase() noexcept {}
    // operators ***************************************************************
    template <typename T>
    void operator=(const Eigen::MatrixBase<T>& vector)
    {
        vector_ = vector;
    }

    // accessor ****************************************************************
    virtual Scalar angle() const { return vector_.norm(); }
    virtual Axis axis() const
    {
        Axis ax = vector_.normalized();

        if (!std::isfinite(ax.sum()))
        {
            return Axis(1, 0, 0);
        }
        return ax;
    }
    virtual AngleAxis angle_axis() const { return AngleAxis(angle(), axis()); }
    virtual Quaternion quaternion() const { return Quaternion(angle_axis()); }
    virtual RotationMatrix rotation_matrix() const
    {
        return RotationMatrix(quaternion());
    }
    virtual EulerVector inverse() const { return EulerVector(-vector_); }
    virtual const Base& vector() const { return vector_; }
    // mutators ****************************************************************
    virtual void angle_axis(const Scalar& angle, const Axis& axis)
    {
        vector_ = angle * axis;

        rescale();
    }
    virtual void angle_axis(const AngleAxis& ang_axis)
    {
        angle_axis(ang_axis.angle(), ang_axis.axis());
    }
    virtual void quaternion(const Quaternion& quat)
    {
        angle_axis(AngleAxis(quat));
    }
    virtual void rotation_matrix(const RotationMatrix& rot_mat)
    {
        angle_axis(AngleAxis(rot_mat));
    }
    virtual void rescale()  // make sure the norm is in [0,Pi]
    {
        Scalar alpha = angle();

        if (alpha <= M_PI)
        {
            return;
        }
        if (alpha > 2 * M_PI)
        {
            alpha -= std::floor(alpha / (2 * M_PI)) * 2 * M_PI;
        }
        if (alpha > M_PI)
        {
            alpha -= 2 * M_PI;
        }

        vector_ = axis() * alpha;
    }

    // operators ***************************************************************
    template <typename T>
    EulerVector operator*(const EulerBase<T>& factor)
    {
        EulerVector product(Base::Zero());
        product.quaternion(this->quaternion() * factor.quaternion());
        return product;
    }
    template <typename T>
    EulerVector operator=(const EulerBase<T>& other)
    {
        vector_ = other.vector();
    }
protected:
    Base vector_;
};

/// implementation for vectors *************************************************
using EulerVector = EulerBase<Eigen::Matrix<Real,3,1>>;

/// implementation for blocks **************************************************
using EulerBlock = EulerBase<Eigen::Ref<Eigen::Matrix<Real,3,1>>>;
}
