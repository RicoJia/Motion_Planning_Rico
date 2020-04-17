#include "../include/prm/rigid2d.hpp"
//#include "rigid2d/rigid2d.hpp"

using std::cout;
using std::cin;
using std::endl;


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v){
    os<<"["<<v.x<<", "<<" "<<v.y<<"]";
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v){
    is>>v.x;
    is>>v.y;
    return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf){
    os<<"******************"<<endl<<tf.T<<endl<<"******************"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf){
    double theta;
    rigid2d::Vector2D v;
    is>>theta;
    is>>v.x;
    is>>v.y;
    rigid2d::Transform2D new_t(v, theta);
    tf = new_t;
    return is;
}

rigid2d::Transform2D::Transform2D(){
    T = MatrixXd::Identity(3, 3);
}

rigid2d::Transform2D::Transform2D(const rigid2d::Vector2D & trans){
    T = MatrixXd::Identity(3, 3);
    T(0,2) = trans.x;
    T(1,2) = trans.y;
}

rigid2d::Transform2D::Transform2D(double radians){
    T = MatrixXd::Identity(3, 3);
    double c = cos(radians);
    double s = sin(radians);
    T(0,0) = c; T(0,1) = -1.0*s;
    T(1,0) = s; T(1,1) = c;
}

rigid2d::Transform2D::Transform2D(const rigid2d::Vector2D &trans, double radians) {
    T = MatrixXd::Identity(3, 3);
    double c = cos(radians);
    double s = sin(radians);
    T(0,0) = c; T(0,1) = -1.0*s; T(0,2) = trans.x;
    T(1,0) = s; T(1,1) = c; T(1,2) = trans.y;
}

rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const {
    Vector3d u(v.x, v.y, 1.0);
    Vector3d u_trans = T*u;
    return Vector2D(u_trans(0), u_trans(1));
}

rigid2d::Transform2D rigid2d::Transform2D::inv() const {
    auto T_inv = T.inverse();
    rigid2d::Vector2D p_inv(T_inv(0,2), T_inv(1,2));
    double theta_inv = -1*atan2(T(1,0), T(0,0));
    rigid2d::Transform2D trans_inv(p_inv, theta_inv);
    return trans_inv;
}

rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const rigid2d::Transform2D & rhs){
    T = T*rhs.T;
    return *this;
}

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D &rhs) {
    return lhs*=rhs;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & v){
    os<<"["<<v.theta<<endl<<" "<<v.x<<endl<<" "<<v.y<<"]"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & v){
    is>>v.theta;
    is>>v.x;
    is>>v.y;
    return is;
}
rigid2d::Twist2D rigid2d::Transform2D::operator()(rigid2d::Twist2D v) const{
    auto adj = T;
    adj(0,2) = T(1,2);
    adj(1,2) = -1*T(0,2);
    Vector3d v_vec(v.x, v.y, v.theta);
    auto new_v = adj*v_vec;
    rigid2d::Twist2D ret_v(new_v(2), new_v(0), new_v(1));
    return ret_v;

}

rigid2d::Twist2D rigid2d::Transform2D::displacement() const{
    double theta = atan2(T(1,0), T(0,0));
    Twist2D twist(theta, T(0,2), T(1,2));
    return twist;
}

rigid2d::Transform2D rigid2d::integrateTwist(const rigid2d::Twist2D& twist){
    auto theta = twist.theta;
    //Cuando theta == 0, se vuelve una transformacion con la misma distancia y 0 angulo.
    if (theta==0){
        rigid2d::Vector2D displacement(twist.x, twist.y);
        Transform2D transform(displacement);
        return transform;
    }
    else{
    // Cuando theta!=0, la matriz rotacional debe ser la que con theta.
        auto vx = twist.x/twist.theta;
        auto vy = twist.y/twist.theta;
        Vector2d v(vx,vy);
        //construir una w matriz para G_theta y la matriz de identitad
        MatrixXd w_hat(2,2);
//        w_hat<<0.0, -1.0*theta, theta, 0.0;
        w_hat<<0.0, -1.0, 1.0, 0.0;
        MatrixXd I = MatrixXd::Identity(2, 2);

        auto G_theta = I*theta+w_hat*(1.0-cos(theta)) + (w_hat*w_hat)*(theta-sin(theta));
        auto v_result = G_theta*v;

        //conseguir la transformacion nueva
        rigid2d::Vector2D v_result_new(v_result(0), v_result(1));
        Transform2D transform( v_result_new, theta);

        return transform;
    }
}

//-------------------------Vector2D Functions
bool rigid2d::Vector2D::operator==(const Vector2D& rhs) const {
    return (x==rhs.x)&&(y==rhs.y);
}
rigid2d::Vector2D& rigid2d::Vector2D::operator-=(const rigid2d::Vector2D& rhs){
    x -= rhs.x; y-=rhs.y;
    return *this;
}

rigid2d::Vector2D& rigid2d::Vector2D::operator+=(const rigid2d::Vector2D& rhs){
    x += rhs.x; y +=rhs.y;
    return *this;
}

rigid2d::Vector2D& rigid2d::Vector2D::operator*=(const double& rhs){
    x *=rhs; y*=rhs;
    return *this;
}

rigid2d::Vector2D rigid2d::operator*(const double& lhs,  rigid2d::Vector2D rhs){
    rhs*=lhs;
    return rhs;
}

rigid2d::Vector2D rigid2d::operator*(rigid2d::Vector2D lhs, const double& rhs){
    lhs*=rhs;
    return lhs;
}

double rigid2d::operator*(rigid2d::Vector2D lhs, rigid2d::Vector2D rhs){
    return lhs.x*rhs.x + lhs.y*rhs.y;
}

rigid2d::Vector2D rigid2d::operator+(rigid2d::Vector2D lhs, const rigid2d::Vector2D& rhs){
    lhs+=rhs;
    return lhs;
}

rigid2d::Vector2D rigid2d::operator-(rigid2d::Vector2D lhs, const rigid2d::Vector2D& rhs){
    lhs-=rhs;
    return lhs;
}

double rigid2d::length(const rigid2d::Vector2D& vec){
    return sqrt(vec.x*vec.x+ vec.y*vec.y);
}

double rigid2d::distance(const rigid2d::Vector2D& vec1, const rigid2d::Vector2D& vec2){
    return sqrt(pow((vec1.x-vec2.x),2)+ pow((vec1.y-vec2.y),2));
}

double rigid2d::angle(const rigid2d::Vector2D& vec){
    return atan2(vec.y, vec.x);
}