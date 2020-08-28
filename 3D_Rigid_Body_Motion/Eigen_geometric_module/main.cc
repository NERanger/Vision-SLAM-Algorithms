#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen geometric module
#include <Eigen/Geometry>

// This program shows the usage of Eigen geometric module

int main(int argc, char const *argv[])
{
    // Eigen/Geometric module provides many kinds of representation for rotation and translation
    // Directly use Matrix3d or Matrix3f for 3D rotation matrix
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // Use AngleAxis for rotation vector (AKA Axis-Angle)
    // The underlying implementation is not Matrix, but we can treat it as a matrix in calculation
    // because there is an operator overloading

    // Rotate 45 degree about z-axis
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d(0, 0, 1) );
    cout.precision(3);
    // Convert to matrix with "matrix()"
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;
    // We can also convert it by "toRotationMatrix" and assign to the variable
    rotation_matrix = rotation_vector.toRotationMatrix();
    // Perform coordinate transformation with AngleAxis
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
    // Perform coordinate transformation with rotation matrix
    v_rotated = rotation_matrix * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

    // Euler angle: convert rotation directly to Euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // Z,X,Y (YAW,PITCH,ROLL)
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // Euclidean transformation matrix using Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // It is a 4x4 matrix although called "3d"
    T.rotate(rotation_vector); // Rotate according to rotation_vector
    T.pretranslate( Eigen::Vector3d(1, 3, 4) ); // Set translation vector to (1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // Perfrom coordinate transformation with transformation matrix
    Eigen::Vector3d v_transformed = T * v; // Equal to R*v+t
    cout << "v transformed = " << v_transformed.transpose() << endl;

    // Use Eigen::Affine3d and Eigen::Projective3d as Affine and Projective transformation

    // Quaternion
    // We can directly assign the value of AngleAxis to quaternion, vice versa
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    cout << "quaternion =\n" << q.coeffs() << endl; // (x,y,z,w) w is real part, xyz is imaginary part
    // We can also assign a rotation matrix to quaternion
    q = Eigen::Quaterniond(rotation_matrix);
    cout << "quaternion =\n" << q.coeffs() << endl;
    // Use the overloaded mutiplication operator to rotate a vector with quaternion
    v_rotated = q * v; // In math it is qvq^{-1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    return 0;
}
