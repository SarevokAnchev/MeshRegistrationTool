//
// Created by Louis on 17/06/2021.
//

#include "MeshRegistration.h"

#include <vtkCenterOfMass.h>

MeshRegistration::MeshRegistration()
{
    transform = Eigen::Matrix4d::Identity(4, 4);
}

MeshRegistration::MeshRegistration(const vtkSmartPointer<vtkPolyData> & fixed_mesh, const vtkSmartPointer<vtkPolyData>& moving_mesh)
    : fixed(fixed_mesh), moving(moving_mesh)
{
    transform = Eigen::Matrix4d::Identity(4, 4);
}

void MeshRegistration::center()
{
    if (!is_valid()) return;
    auto filter = vtkSmartPointer<vtkCenterOfMass>::New();

    filter->SetInputData(fixed);
    filter->Update();
    double fixed_center[3];
    filter->GetCenter(fixed_center);

    auto moved = apply(moving);
    filter->SetInputData(moved);
    filter->Update();
    double moving_center[3];
    filter->GetCenter(moving_center);

    std::cout << "Moving mesh to ("
        << fixed_center[0] << ","
        << fixed_center[1] << ","
        << fixed_center[2] << ")" << std::endl;

    Eigen::Vector4d distance(
        fixed_center[0] - moving_center[0],
        fixed_center[1] - moving_center[1],
        fixed_center[2] - moving_center[2],
        1
    );
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.col(3) << distance.col(0);
    transform = mat*transform;
}

vtkSmartPointer<vtkPolyData> MeshRegistration::apply(const vtkSmartPointer<vtkPolyData>& mesh) const
{
    if (!is_valid()) return nullptr;
    auto new_mesh = vtkSmartPointer<vtkPolyData>::New();
    new_mesh->DeepCopy(mesh);

    auto n_points = new_mesh->GetNumberOfPoints();
    auto points = new_mesh->GetPoints();
    Eigen::MatrixXd verts(4, n_points);
    for (long long i = 0; i < n_points; i++) {
        double p[3];
        points->GetPoint(i, p);
        verts(0, i) = p[0];
        verts(1, i) = p[1];
        verts(2, i) = p[2];
        verts(3, i) = 1.;
    }

    verts = transform*verts;

    for (long long i = 0; i < n_points; i++) {
        double p[3];
        p[0] = verts(0, i);
        p[1] = verts(1, i);
        p[2] = verts(2, i);
        points->SetPoint(i, p);
    }
    return new_mesh;
}
