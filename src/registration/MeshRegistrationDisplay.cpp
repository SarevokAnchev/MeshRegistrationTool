//
// Created by Louis on 21/06/2021.
//

#include "MeshRegistrationDisplay.h"

#include <iostream>

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPointData.h>
#include <Eigen/Dense>

#include "../utils/mesh.h"

MeshRegistrationDisplay::MeshRegistrationDisplay(const vtkSmartPointer<vtkPolyData>& fixed_mesh, const vtkSmartPointer<vtkPolyData>& moving_mesh)
{
    fixed = vtkSmartPointer<vtkPolyData>::New();
    fixed->DeepCopy(fixed_mesh);
    fixed->GetPointData()->SetScalars(nullptr);
    moving = vtkSmartPointer<vtkPolyData>::New();
    moving->DeepCopy(moving_mesh);
    moving->GetPointData()->SetScalars(nullptr);
    transform = Eigen::Matrix4d::Identity(4, 4);
    center_of_mass(moving, moving_center_of_mass);
}

void MeshRegistrationDisplay::run()
{
    std::cout << "Mesh Registration Display:" << std::endl;
    std::cout << "Arrow keys: Translations" << std::endl;
    std::cout << "PageUp / PageDown: Rotations" << std::endl;
    std::cout << "A / Z: Scaling" << std::endl;
    std::cout << "F / D and V / C: fixed and moving mesh opacity" << std::endl;
    std::cout << "Q: quit and save transform" << std::endl;

    auto fixed_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    fixed_mapper->SetInputData(fixed);
    fixed_actor = vtkSmartPointer<vtkActor>::New();
    fixed_actor->SetMapper(fixed_mapper);
    fixed_actor->GetProperty()->SetOpacity(0.6);
    fixed_actor->GetProperty()->SetColor(0.2, 0.7, 0.3);

    auto moving_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    moving_mapper->SetInputData(apply_transform(moving));
    moving_actor = vtkSmartPointer<vtkActor>::New();
    moving_actor->SetMapper(moving_mapper);
    moving_actor->GetProperty()->SetOpacity(0.6);
    moving_actor->GetProperty()->SetColor(0.7, 0.2, 0.6);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(fixed_actor);
    renderer->AddActor(moving_actor);
    renderer->SetBackground(1., 1., 1.);
    renderer->ResetCamera();
    renderer->GetActiveCamera()->SetParallelProjection(true);

    renwin = vtkSmartPointer<vtkRenderWindow>::New();
    renwin->AddRenderer(renderer);
    renwin->SetSize(1024, 768);

    interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
    interactor->SetRenderWindow(renwin);

    // CALLBACKS
    auto move_callback = vtkSmartPointer<vtkCallbackCommand>::New();
    move_callback->SetCallback(cb_key_pressed);
    move_callback->SetClientData(this);
    interactor->AddObserver(vtkCommand::KeyPressEvent, move_callback);
    renwin->Render();
    interactor->Start();
}

vtkSmartPointer<vtkPolyData> MeshRegistrationDisplay::apply_transform(vtkSmartPointer<vtkPolyData> mesh)
{
    auto n_points = mesh->GetNumberOfPoints();
    Eigen::MatrixXd positions(4, n_points);
    for (long long i = 0; i < n_points; i++) {
        double p[3];
        mesh->GetPoint(i, p);
        positions(0, i) = p[0];
        positions(1, i) = p[1];
        positions(2, i) = p[2];
        positions(3, i) = 1;
    }

    auto new_positions = transform*positions;

    auto new_mesh = vtkSmartPointer<vtkPolyData>::New();
    new_mesh->DeepCopy(mesh);
    auto points = new_mesh->GetPoints();
    for (long long i = 0; i < n_points; i++) {
        double p[3];
        p[0] = new_positions(0, i);
        p[1] = new_positions(1, i);
        p[2] = new_positions(2, i);
        points->SetPoint(i, p);
    }
    return new_mesh;
}

void MeshRegistrationDisplay::key_pressed()
{
    std::string code(interactor->GetKeySym());
    if (code == "Up" || code == "Down" || code == "Left" || code == "Right") {
        translation(code);
    }
    else if (code == "Prior" || code == "Next") {
        rotation(code);
    }
    else if (code == "a" || code == "z") {
        scaling(code);
    }
    else if (code == "f") change_opacity(OPACITY_COMMAND::FIXED_UP);
    else if (code == "d") change_opacity(OPACITY_COMMAND::FIXED_DOWN);
    else if (code == "v") change_opacity(OPACITY_COMMAND::MOVING_UP);
    else if (code == "c") change_opacity(OPACITY_COMMAND::MOVING_DOWN);
}

void MeshRegistrationDisplay::translation(const std::string & direction)
{
    auto* cam = renderer->GetActiveCamera();
    double axial[3];
    cam->GetDirectionOfProjection(axial);
    Eigen::Vector3d cam_axial(axial);
    cam->OrthogonalizeViewUp();
    double up[3];
    cam->GetViewUp(up);
    Eigen::Vector3d cam_up(up);
    Eigen::Vector3d cam_side = cam_axial.cross(cam_up);

    auto vtk_trans = vtkSmartPointer<vtkTransform>::New();
    auto init_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int r = 0; r < transform.rows(); r++) {
        for (int c = 0; c < transform.cols(); c++) {
            init_matrix->SetElement(r, c, transform(r, c));
        }
    }
    vtk_trans->SetMatrix(init_matrix);

    Eigen::Vector3d displacement;
    if (direction == "Up")
        displacement = cam_up.normalized();
    else if (direction == "Down")
        displacement = -cam_up.normalized();
    else if (direction == "Left")
        displacement = -cam_side.normalized();
    else if (direction == "Right")
        displacement = cam_side.normalized();

    vtk_trans->PostMultiply();
    vtk_trans->Translate(displacement(0), displacement(1), displacement(2));

    renderer->RemoveActor(moving_actor);
    auto t_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    t_filter->SetInputData(moving);
    t_filter->SetTransform(vtk_trans);
    t_filter->Update();
    vtkSmartPointer<vtkPolyData> result(t_filter->GetOutput());
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(result);
    moving_actor->SetMapper(mapper);
    renderer->AddActor(moving_actor);

    auto* mat = vtk_trans->GetMatrix();
    Eigen::Matrix4d rot_transform;
    for (int r = 0; r < rot_transform.rows(); r++) {
        for (int c = 0; c < rot_transform.cols(); c++) {
            rot_transform(r, c) = mat->GetElement(r, c);
        }
    }
    transform = rot_transform;
    renwin->Render();
}

void MeshRegistrationDisplay::rotation(const std::string & direction)
{
    auto vtk_trans = vtkSmartPointer<vtkTransform>::New();
    auto init_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int r = 0; r < transform.rows(); r++) {
        for (int c = 0; c < transform.cols(); c++) {
            init_matrix->SetElement(r, c, transform(r, c));
        }
    }
    vtk_trans->SetMatrix(init_matrix);

    auto* cam = renderer->GetActiveCamera();
    double axis[3];
    cam->GetDirectionOfProjection(axis);
    double rot_dir = 1.;
    if (direction == "Next") rot_dir = -1;
    double focal[3];
    cam->GetFocalPoint(focal);

    vtk_trans->PostMultiply();
    vtk_trans->Translate(-focal[0], -focal[1], -focal[2]);
    vtk_trans->RotateWXYZ(rot_dir, axis);
    vtk_trans->Translate(focal[0], focal[1], focal[2]);

    renderer->RemoveActor(moving_actor);
    auto t_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    t_filter->SetInputData(moving);
    t_filter->SetTransform(vtk_trans);
    t_filter->Update();
    vtkSmartPointer<vtkPolyData> result(t_filter->GetOutput());
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(result);
    moving_actor->SetMapper(mapper);
    renderer->AddActor(moving_actor);

    auto* mat = vtk_trans->GetMatrix();
    Eigen::Matrix4d rot_transform;
    for (int r = 0; r < rot_transform.rows(); r++) {
        for (int c = 0; c < rot_transform.cols(); c++) {
            rot_transform(r, c) = mat->GetElement(r, c);
        }
    }
    transform = rot_transform;
    renwin->Render();
}

void MeshRegistrationDisplay::change_opacity(OPACITY_COMMAND c)
{
    auto moving_opacity = moving_actor->GetProperty()->GetOpacity();
    auto fixed_opacity = fixed_actor->GetProperty()->GetOpacity();
    switch (c) {
        case OPACITY_COMMAND::FIXED_UP:
            fixed_opacity += 0.1;
            break;
        case OPACITY_COMMAND::FIXED_DOWN:
            fixed_opacity -= 0.1;
            break;
        case OPACITY_COMMAND::MOVING_UP:
            moving_opacity += 0.1;
            break;
        case OPACITY_COMMAND::MOVING_DOWN:
            moving_opacity -= 0.1;
            break;
    }
    if (moving_opacity < 0) moving_opacity = 0;
    if (moving_opacity > 1) moving_opacity = 1;
    if (fixed_opacity < 0) fixed_opacity = 0;
    if (fixed_opacity > 1) fixed_opacity = 1;

    moving_actor->GetProperty()->SetOpacity(moving_opacity);
    fixed_actor->GetProperty()->SetOpacity(fixed_opacity);

    renwin->Render();
}

void MeshRegistrationDisplay::scaling(const std::string & code)
{
    auto vtk_trans = vtkSmartPointer<vtkTransform>::New();
    auto init_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int r = 0; r < transform.rows(); r++) {
        for (int c = 0; c < transform.cols(); c++) {
            init_matrix->SetElement(r, c, transform(r, c));
        }
    }
    vtk_trans->SetMatrix(init_matrix);

    float scale[] = {1., 1., 1.};
    if (code == "a") {
        scale[0] = 1.01;
        scale[1] = 1.01;
        scale[2] = 1.01;
    }
    else {
        scale[0] = 0.99;
        scale[1] = 0.99;
        scale[2] = 0.99;
    }
    auto mesh_center = vtk_trans->TransformDoublePoint(moving_center_of_mass);

    vtk_trans->PostMultiply();
    vtk_trans->Translate(-mesh_center[0], -mesh_center[1], -mesh_center[2]);
    vtk_trans->Scale(scale);
    vtk_trans->Translate(mesh_center[0], mesh_center[1], mesh_center[2]);

    renderer->RemoveActor(moving_actor);
    auto t_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    t_filter->SetInputData(moving);
    t_filter->SetTransform(vtk_trans);
    t_filter->Update();
    vtkSmartPointer<vtkPolyData> result(t_filter->GetOutput());
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(result);
    moving_actor->SetMapper(mapper);
    renderer->AddActor(moving_actor);

    auto* mat = vtk_trans->GetMatrix();
    Eigen::Matrix4d scale_tfm;
    for (int r = 0; r < scale_tfm.rows(); r++) {
        for (int c = 0; c < scale_tfm.cols(); c++) {
            scale_tfm(r, c) = mat->GetElement(r, c);
        }
    }
    transform = scale_tfm;
    renwin->Render();
}

void cb_key_pressed(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
    auto window = (MeshRegistrationDisplay*)clientData;
    window->key_pressed();
}
