#pragma once
// Rigid transform, as Sophus::SE3 (which ORB-SLAM3 uses for every pose): a unit quaternion and a translation,
// with Sophus's operations, so poses compose and round as there.
//   product      quaternion product written out, then normalized (SO3's quaternion constructor)
//   inverse      conjugate (normalized), translation -R⁻¹t
//   R * p        p + w·uv + v × uv, uv = 2 v × p (Sophus's group action, not the matrix)
//   from R       Eigen's quaternion from the matrix, not normalized (Sophus's SO3(Matrix3))
//   rotation()   the quaternion's rotation matrix
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace geometry {

template <class S>
class SE3 {
public:
    using Vector3 = Eigen::Matrix<S, 3, 1>;
    using Matrix3 = Eigen::Matrix<S, 3, 3>;
    using Matrix4 = Eigen::Matrix<S, 4, 4>;
    using Quaternion = Eigen::Quaternion<S>;

    SE3() : q_(Quaternion::Identity()), t_(Vector3::Zero()) {}
    // R must be a rotation (orthonormal, det 1)
    SE3(const Matrix3& R, const Vector3& t) : q_(R), t_(t) {}
    SE3(const Quaternion& q, const Vector3& t) : q_(q), t_(t) { normalize(); }
    // The upper-left 3x3 must be a rotation
    explicit SE3(const Matrix4& T) : q_(Matrix3(T.template topLeftCorner<3, 3>())), t_(T.template block<3, 1>(0, 3)) {}

    const Quaternion& unitQuaternion() const { return q_; }
    Matrix3 rotationMatrix() const { return q_.toRotationMatrix(); }
    const Vector3& translation() const { return t_; }
    Vector3& translation() { return t_; }
    Matrix4 matrix() const {
        Matrix4 T = Matrix4::Identity();
        T.template topLeftCorner<3, 3>() = rotationMatrix();
        T.template block<3, 1>(0, 3) = t_;
        return T;
    }
    Eigen::Matrix<S, 3, 4> matrix3x4() const {
        Eigen::Matrix<S, 3, 4> T;
        T.template leftCols<3>() = rotationMatrix();
        T.col(3) = t_;
        return T;
    }

    // The rotation applied to p
    Vector3 rotate(const Vector3& p) const {
        Vector3 uv = q_.vec().cross(p);
        uv += uv;
        return p + q_.w() * uv + q_.vec().cross(uv);
    }
    Vector3 operator*(const Vector3& p) const { return rotate(p) + t_; }

    SE3 operator*(const SE3& o) const {
        const Quaternion& a = q_;
        const Quaternion& b = o.q_;
        SE3 r;
        r.q_ = Quaternion(a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
                          a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
                          a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
                          a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x());
        r.normalize();
        r.t_ = t_ + rotate(o.t_);
        return r;
    }

    SE3 inverse() const {
        SE3 r;
        r.q_ = q_.conjugate();
        r.normalize();
        r.t_ = r.rotate(-t_);
        return r;
    }

    template <class T>
    SE3<T> cast() const {
        SE3<T> r;
        r = SE3<T>(q_.template cast<T>(), t_.template cast<T>());
        return r;
    }

private:
    void normalize() { q_.coeffs() /= q_.coeffs().norm(); }

    Quaternion q_;
    Vector3 t_;
};

using SE3f = SE3<float>;
using SE3d = SE3<double>;

}  // namespace geometry
