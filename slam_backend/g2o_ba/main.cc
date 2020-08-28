#include <iostream>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>

#include <sophus/se3.hpp>

#include "common.hpp"

using namespace Sophus;
using namespace Eigen;
using namespace std;

// Struct for pose and intrinsics
struct PoseAndIntrinsics{
    PoseAndIntrinsics() {}

    // Set form given data address
    explicit PoseAndIntrinsics(double *data_addr) {
        rotation = SO3d::exp(Vector3d(data_addr[0], data_addr[1], data_addr[2]));
        translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);
        focal = data_addr[6];
        k1 = data_addr[7];
        k2 = data_addr[8];
    }

    // Put estimated value into RAM
    void set_to(double *data_addr) {
        auto r = rotation.log();
        for (int i = 0; i < 3; i++) {data_addr[i] = r[i];}
        for (int i = 0; i < 3; i++) {data_addr[i + 3] = translation[i];}
        data_addr[6] = focal;
        data_addr[7] = k1;
        data_addr[8] = k2;
    }

    SO3d rotation;
    Vector3d translation = Vector3d::Zero();
    double focal = 0;
    double k1 = 0, k2 = 0;
};

// Vertex for pose and camera intrinsics
// 9 dimension in total: 3 for so3, 3 for t, and f, k1, k2 
class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPoseAndIntrinsics() {}

        virtual void setToOriginImpl() override {
            _estimate = PoseAndIntrinsics();
        }

        virtual void oplusImpl(const double *update) override {
            _estimate.rotation = SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
            _estimate.translation += Vector3d(update[3], update[4], update[5]);
            _estimate.focal += update[6];
            _estimate.k1 += update[7];
            _estimate.k2 += update[8];
        }

        // Project a point according to the estimation
        Vector2d project(const Vector3d &point) {
            Vector3d pc = _estimate.rotation * point + _estimate.translation;
            pc = - pc / pc[2];
            double r2 = pc.squaredNorm();
            double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);

            return Vector2d(_estimate.focal * distortion * pc[0], _estimate.focal * distortion * pc[1]);
        }

        virtual bool read(istream &in) {}
        virtual bool write(ostream &out) const {}

};

class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPoint() {}

        virtual void setToOriginImpl() override {
            _estimate = Vector3d(0, 0, 0);
        }

        virtual void oplusImpl(const double *update) override {
            _estimate += Vector3d(update[0], update[1], update[2]);
        }

        virtual bool read(istream &in) {}

        virtual bool write(ostream &out) {}
};

class EdgeProjection : public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics, VertexPoint> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void computeError() override {
            auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
            auto v1 = (VertexPoint *) _vertices[1];
            auto proj = v0->project(v1->estimate());
            _error = proj - _measurement;
        }

        virtual bool read(istream &in) {}

        virtual bool write(ostream &out) const {}
};

void SloveBA(BALProblem &bal_problem);

int main(int argc, char **argv){
    
}