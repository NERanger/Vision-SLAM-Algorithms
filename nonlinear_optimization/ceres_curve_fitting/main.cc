#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// The computing model for cost function
struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    template<typename T>
    bool operator()(
        const T *const abc,   // model parameter, 3 dimension
        T *residual) const {
            // y-exp(ax^2+bx+c) abc[0] = a; abc[1] = b; abc[2] = c;
            residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
            return true;
    }

    const double _x, _y; // data for x, y
};

int main(int argc, char **argv){
    double ar = 1.0, br = 2.0, cr = 1.0;   // Groundtruth parameter value
    double ae = 2.0, be = -1.0, ce = 5.0;  // Estimated(initial) parameter value
    int N = 100;                           // Number of data points
    double w_sigma = 1.0;                  // Sigma value for noise
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                           // OpenCV random number generator

    vector<double> x_data, y_data;         // Data
    for(int i = 0; i < N; i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {ae, be, ce};

    // Construct the least squares problem
    ceres::Problem problem;
    for(int i = 0; i < N; i++){
        problem.AddResidualBlock(  // Add residual block into the problem
            // Use the auto-differential feature
            // Template parameter: error type, output dimension, input dimension
            // Input dimension should be the same with that defined in the struct at the beginning
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,  // Kernel function
            abc       // The parameters to be estimated
        );
    }

    // Config the solver
    ceres::Solver::Options options;  // Here you can config many settings
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // Solution method for incremental equation
    options.minimizer_progress_to_stdout = true;  // Output to cout

    ceres::Solver::Summary summary;  // Optimization info
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // Begin the optimization
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "Solve time cost = " << time_used.count() << " seconds. " << endl;

    // Output the result
    cout << summary.BriefReport() << endl;
    cout << "Estimated a, b, c = ";
    for(auto a:abc) cout << a << " ";
    cout << endl;

    return 0;
}
