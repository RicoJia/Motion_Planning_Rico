#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <Eigen/Dense>
#include <math.h>       /* cos */

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector2d;

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-10)
    {
        return ((d1-d2)*(d1-d2)<epsilon*epsilon);
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return deg*PI/180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad*180.0/PI;
    }

    /// \brief convert radians into [-pi, pi]
    /// \param rad - angle in radians
    /// \returns normalized angle in radians
    constexpr double normalize_angle(double rad)
    {
        auto _rad = rad - ((int)(rad/(2*PI)))*(2*PI);       //pow, mod functions are not available during compile time
        if (abs(_rad) > PI)
            _rad = (_rad>0)?-1.0*(2.0*PI-_rad):2.0*PI+_rad;
        if (_rad == -1*PI)
            _rad = PI;
        return _rad;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens.
    /// The if almost_equal(a,b) returns false, then static_assert will pop up an error msg in compile time.
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal( normalize_angle(0.0),0.0), "normalized angle failed");      //I wrote a test case for normalized angle.
    static_assert(almost_equal( normalize_angle(PI),PI), "normalized angle failed");      //I wrote a test case for normalized angle.


    /// \brief A circle with (x,y) as center and radius
    struct Circle{
        double x, y;
        double radius;
        Circle(double x, double y, double radius): x(x), y(y), radius(radius){}
    };

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x,y;
        Vector2D(double x, double y):x(x), y(y){}
        Vector2D():x(0.0), y(0.0){}
        /// \brief Normalize this vector
        void normalize_vec(){
            double temp_x, temp_y;
            temp_x = x/pow((x*x+y*y),0.5);
            temp_y = y/pow((x*x+y*y),0.5);
            x = temp_x;
            y = temp_y;
        }
        /// \brief compose self subtraction transform with another and store the result
        /// \param rhs - the first Vector2D to apply
        /// \returns a reference to the newly transformed Vector2D
        Vector2D& operator-=(const Vector2D& rhs);

        /// \brief compose self addition transform with another and store the result
        /// \param rhs - the first Vector2D to apply
        /// \returns a reference to the newly transformed Vector2D
        Vector2D& operator+=(const Vector2D& rhs);

        /// \brief compose self multiplication with scalar transform with another and store the result
        /// \param rhs - the first Vector2D to apply
        /// \returns a reference to the newly transformed Vector2D
        Vector2D& operator*=(const double& rhs);

        /// \brief: if two vectors have all same components
        bool operator==(const Vector2D& rhs) const;
    };

    /// \brief multiplication with scalar (scalar on the left) and store the result
    /// \param rhs - two Vector2D objects
    /// \returns a copy to the newly transformed Vector2D
    Vector2D operator*(const double& lhs, Vector2D rhs);

    /// \brief multiplication with scalar (scalar on the right) and store the result
    /// \param rhs - two Vector2D objects
    /// \returns a copy to the newly transformed Vector2D
    Vector2D operator*(Vector2D lhs, const double& rhs);

    /// \brief multiplication with another Vector2D and store the result
    /// \param rhs - double
    /// \returns a copy to the newly transformed Vector2D
    double operator*(Vector2D lhs, Vector2D rhs);

    /// \brief addition with another Vector2D and store the result
    /// \param rhs - two Vector2D objectsapply
    /// \returns a copy to the newly transformed Vector2D
    Vector2D operator+(Vector2D lhs, const Vector2D& rhs);

    /// \brief addition with another Vector2D and store the result
    /// \param rhs - two Vector2D objects
    /// \returns a copy to the newly transformed Vector2D
    Vector2D operator-(Vector2D lhs, const Vector2D& rhs);


    /// \brief Get the length of a vector and store the result
    /// \param rhs - one Vector2D objects
    /// \returns length value
    double length(const Vector2D& vec);

    /// \brief Get the distance between two vectors
    /// \param rhs - two Vector2D objects
    /// \returns distance value
    double distance(const Vector2D& vec1, const Vector2D& vec2);
////    angle(Vector2D) (compute the angle of the vector)

    /// \brief Get the angle between a Vector2D and positive X axis, between -PI and PI.
    /// \param one Vector2D object
    /// \returns angle value
    double angle(const Vector2D& vec1);

//--------------------------------------------------------------------------------------------------------------
    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        double theta, x,y;
        Twist2D(double theta = 0, double x = 0, double y = 0):theta(theta), x(x), y(y){}
    };

/// \brief output a 2 dimensional twist as [theta, xcomponent ycomponent]
/// os - stream to output to
/// v - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & v);

/// \brief input a 2 dimensional twist
///   You should be able to read twists entered as two numbers
///   separated by a newline or a space, or entered as [theta, xcomponent ycomponent]
/// is - stream from which to read
/// v [out] - output twist
/// Hint: The following may be useful:
/// https://en.cppreference.com/w/cpp/io/basic_istream/peek
/// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Twist2D & v);


//    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    /// \brief \see operator<<(...) (declared outside this class)
    /// for a description
    friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
//    friend std::istream & operator>>(std::istream & is, Transform2D & tf);

    public:
        /// \brief Create an 3x3 identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        explicit Transform2D(const Vector2D & trans, double radians);
//
        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;
//
        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief return theta, x, y in a Twist2D object
        /// \returns a Twist2D object
        Twist2D displacement() const;       //TODO: check math

    private:
        MatrixXd T;
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief compute the transformation corresponding to a rigid body following a constant twist for one time unit
    /// \param Twist2D& - ttwist in unit time [theta, x, y], which is screw x theta. screw is [1, vx, vy].
    /// \return Transform2D SE2
    Transform2D integrateTwist(const Twist2D&);

    class DiffDrive;
}

#endif
