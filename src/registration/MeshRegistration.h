//
// Created by Louis on 17/06/2021.
//

#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <Eigen/Dense>

class MeshRegistration {
private:
    vtkSmartPointer<vtkPolyData> fixed;
    vtkSmartPointer<vtkPolyData> moving;

    Eigen::Matrix4d transform;

public:
    MeshRegistration();
    MeshRegistration(const vtkSmartPointer<vtkPolyData>&, const vtkSmartPointer<vtkPolyData>&);

    void center();

    inline void set_fixed(const vtkSmartPointer<vtkPolyData>& mesh) { fixed = mesh; }

    inline vtkSmartPointer<vtkPolyData> get_fixed() const { return fixed; }

    inline void set_moving(const vtkSmartPointer<vtkPolyData>& mesh) { moving = mesh; }

    inline vtkSmartPointer<vtkPolyData> get_moving() const { return moving; }

    inline Eigen::Matrix4d get_transform() const { return transform; }

    inline void set_transform(Eigen::Matrix4d mat) { transform = std::move(mat); }

    vtkSmartPointer<vtkPolyData> apply(const vtkSmartPointer<vtkPolyData>&) const;

    inline bool is_valid() const { return fixed && moving; }
};
