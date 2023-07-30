//
// Created by Louis on 21-Jun-22.
//

#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>

struct ICPParams {
    bool init_center;
    int max_iterations;
    double tolerance;
};

struct ICPStatus;

class ICP {
public:
    using TransformMatrixType = Eigen::Matrix4d;
    using PointCloudType = std::vector<std::vector<double>>;

    ICP(PointCloudType fixed_, PointCloudType moving_);
    ICP(PointCloudType fixed_, PointCloudType moving_, TransformMatrixType initial_tfm);

    inline void set_transform(TransformMatrixType tfm) { transform = std::move(tfm); }
    [[nodiscard]] inline TransformMatrixType get_transform() const { return transform; }

    inline void set_icp_params(ICPParams params_) { params = params_; }
    [[nodiscard]] inline ICPParams get_icp_params() const { return params; }

    inline void add_callback(std::function<void(ICPStatus)> f) { callbacks.emplace_back(f); }

    void run();

    static TransformMatrixType best_transform(Eigen::Matrix3Xd fixed_, Eigen::Matrix3Xd moving_);

private :
    PointCloudType fixed;
    PointCloudType moving;

    TransformMatrixType transform;

    ICPParams params{};

    std::vector<std::function<void(ICPStatus)>> callbacks;
};

struct ICPStatus {
    int iteration;
    double error;
    double moving_x, moving_y, moving_z;
    ICPParams params;
    ICP::TransformMatrixType tfm;
};

ICP::TransformMatrixType icp_registration(ICP::PointCloudType fixed, ICP::PointCloudType moving, ICPParams params);
