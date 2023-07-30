//
// Created by Louis on 21/06/2021.
//

#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkActor.h>

#include <Eigen/Dense>

enum class OPACITY_COMMAND {
    MOVING_DOWN, MOVING_UP, FIXED_DOWN, FIXED_UP
};

class MeshRegistrationDisplay {
    // TODO : option de reset de la transformation
private:
    vtkSmartPointer<vtkPolyData> fixed;
    vtkSmartPointer<vtkPolyData> moving;

    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    vtkSmartPointer<vtkRenderWindow> renwin;
    vtkSmartPointer<vtkRenderer> renderer;

    vtkSmartPointer<vtkActor> fixed_actor;
    vtkSmartPointer<vtkActor> moving_actor;

    Eigen::Matrix4d transform;

    vtkSmartPointer<vtkPolyData> apply_transform(vtkSmartPointer<vtkPolyData>);

    double moving_center_of_mass[3];

public:
    MeshRegistrationDisplay(const vtkSmartPointer<vtkPolyData>&, const vtkSmartPointer<vtkPolyData>&);

    void run();

    inline void set_transform(const Eigen::Matrix4d& t) { transform = t; }

    [[nodiscard]] inline Eigen::Matrix4d get_transform() const { return transform; }

    void key_pressed();

    void translation(const std::string&);

    void rotation(const std::string&);

    void scaling(const std::string&);

    void change_opacity(OPACITY_COMMAND c);
};

void cb_key_pressed(vtkObject*, long unsigned int, void*, void*);
