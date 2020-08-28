#include <iostream>
#include <ctime>
using namespace std;

//Eigen
#include <Eigen/Core>
//Dense matrix computation
#include <Eigen/Dense>

#define MARTIX_SIZE 50

//This program shows the usage of eigen basic type

int main(int argc, char const *argv[])
{
    // Declare a 2x3 float matrix
    Eigen::Matrix<float,2,3> matrix_23;
    // Eigen provide many built-in type via typedef, but their base is still Eigen::Matrix
    // For example Vector3d is actually Eigen::Matrix<double,3,1>
    Eigen::Vector3d v_3d;
    // Matrix3d is acutally Eigen::Matrix<double,3,3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //Initialize as 0 matrix
    // Dynamic matrix can be applied if the size of the matrix is uncertain
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // Easier declaration
    Eigen::MatrixXd matrix_x;
    // There are a lot of types like the above, we do not list one by one here

    // Matrix operation
    // Data input
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // Data output
    cout << matrix_23 << endl;

    // Use () to access the matrix elements
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            cout << matrix_23(i,j) << " i=" << i << " j=" << j << endl;
        }
    }

    v_3d << 3, 2, 1;
    // Matrix mutiplication
    // But here you cannot mix 2 matrixes of different data types (the following is wrong)
    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;

    // Explicit transform
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    // Matrix demension should be right
    // The following is a wrong example
    //Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    // Some matrix operation 
    // +,-,*,/ not included, which can be directly used
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;

    cout << matrix_33.transpose() << endl; // Transpose
    cout << matrix_33.sum() << endl;       // Sum up all elements
    cout << matrix_33.trace() << endl;     // Matrix trace
    cout << 10 * matrix_33 << endl;        // Matrix mutiplication by single number
    cout << matrix_33.inverse() << endl;   // Matrix inverse
    cout << matrix_33.determinant() << endl;// Matrix determinant

    // Eigenvalues
    // Real symmetric matrix can ensure the success of diagonalization
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33.transpose() * matrix_33);
    cout << "Eigen values = " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = " << eigen_solver.eigenvectors() << endl;

    // Slove a euqation matrix_MN * x = v_Nd
    // N is pre-defined in the macro in the beginning, the matrix is generate randomly
    // Solve the inverse is the most direct way, but it comes with large computation
    Eigen::Matrix<double, MARTIX_SIZE, MARTIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MARTIX_SIZE, MARTIX_SIZE);
    Eigen::Matrix<double, MARTIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MARTIX_SIZE, 1);

    clock_t time_stt = clock(); // Time counter

    // Directly calcute inverse
    Eigen::Matrix<double, MARTIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time use in normal inverse is " << 1000 * (clock() - time_stt) / (double) \
    CLOCKS_PER_SEC << "ms" << endl;

    // Normally we use matrix decomposition for solution, QR decomposition for example, and the speed
    // will be a lot faster
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in QR decomposition is " << 1000 * (clock() - time_stt) / (double) \
    CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}
