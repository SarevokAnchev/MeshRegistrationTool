//
// Created by Louis on 21-Jun-22.
//

#include "ICP.h"

#include <utility>
#include <iostream>

#include "../utils/KDTree.h"

ICP::ICP(PointCloudType fixed_, PointCloudType moving_)
    : fixed(std::move(fixed_)), moving(std::move(moving_))
{
    transform = TransformMatrixType();
    transform.setIdentity();
    params = {
        true, 150, 0.0001
    };
}

ICP::ICP(PointCloudType fixed_, PointCloudType moving_, TransformMatrixType initial_tfm)
    : fixed(std::move(fixed_)), moving(std::move(moving_)), transform(std::move(initial_tfm))
{
    params = {
        true, 150, 0.0001
    };
}

void ICP::run()
{
    KDTree<size_t> fixed_tree;
    std::vector<size_t> indices(fixed.size());
    for (size_t i = 0; i < fixed.size(); i++)
        indices[i] = i;
    fixed_tree.add_nodes(fixed, indices, true);
    Eigen::Matrix4Xd moving_mat;
    moving_mat.resize(4, moving.size());
    for (auto i = 0; i < moving.size(); i++) {
        moving_mat(0, i) = moving[i][0];
        moving_mat(1, i) = moving[i][1];
        moving_mat(2, i) = moving[i][2];
        moving_mat(3, i) = 1;
    }
    moving_mat = transform * moving_mat;

    Eigen::Matrix3Xd m;
    m.resize(3, moving.size());
    Eigen::Matrix3Xd f;
    f.resize(3, moving.size());
    Eigen::Matrix4d cur_tfm;
    cur_tfm = transform;

    auto error = 0.;

    ICPStatus status {
        0,
        0.,
        0, 0, 0,
        params,
        cur_tfm
    };
    for (auto i = 0; i < params.max_iterations; i++) {
        status.iteration = i;
        auto new_error = 0.;
        for (int j = 0; j < moving.size(); j++) {
            std::vector<double> moving_point = {
                moving_mat(0, j),
                moving_mat(1, j),
                moving_mat(2, j)
            };
            m(0, j) = moving_point[0];
            m(1, j) = moving_point[1];
            m(2, j) = moving_point[2];
            auto nearest = fixed_tree.get_closest_node(moving_point);
            f(0, j) = nearest->coord(0);
            f(1, j) = nearest->coord(1);
            f(2, j) = nearest->coord(2);
            new_error += nearest->dist(moving_point);
        }
        new_error /= (double)moving.size();
        if (i > 0 && error - new_error < params.tolerance) {
            // break;
        }
        error = new_error;
        status.error = error;
        auto best = best_transform(f, m);
        cur_tfm = best * cur_tfm;
        status.tfm = cur_tfm;
        moving_mat = best * moving_mat;
        for (const auto& cb : callbacks) {
            cb(status);
        }
    }
    transform = cur_tfm;
}

ICP::TransformMatrixType ICP::best_transform(Eigen::Matrix3Xd fixed_, Eigen::Matrix3Xd moving_)
{
    TransformMatrixType res;
    res.setIdentity();
    auto cf = fixed_.rowwise().mean();
    auto cm = moving_.rowwise().mean();
    Eigen::Matrix3Xd moving_c;
    moving_c.resize(3, moving_.cols());
    Eigen::Matrix3Xd fixed_c;
    fixed_c.resize(3, fixed_.cols());
    for (auto i = 0; i < moving_c.cols(); i++) {
        fixed_c.col(i) = fixed_.col(i) - cf;
        moving_c.col(i) = moving_.col(i) - cm;
    }

    auto h = moving_c * fixed_c.transpose();
    auto h_svd = h.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto r = h_svd.matrixV() * h_svd.matrixU().transpose();
    auto t = cf - r*cm;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res(i, j) = r(i, j);
        }
    }
    res(0, 3) = t(0);
    res(1, 3) = t(1);
    res(2, 3) = t(2);
    return res;
}

ICP::TransformMatrixType icp_registration(ICP::PointCloudType fixed, ICP::PointCloudType moving, ICPParams params)
{
    ICP icp(std::move(fixed), std::move(moving));
    icp.set_icp_params(params);
    icp.add_callback([](ICPStatus s){ std::cout << s.iteration << " - " << s.error << std::endl; });
    icp.run();
    return icp.get_transform();
}
