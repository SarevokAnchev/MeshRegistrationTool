#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkImageData.h>

#include <itkPoint.h>

vtkSmartPointer<vtkPolyData> marching_cubes(const vtkSmartPointer<vtkImageData>&);

vtkSmartPointer<vtkPolyData> simplify_mesh(const vtkSmartPointer<vtkPolyData>&, float);

vtkSmartPointer<vtkPolyData> extract_largest(vtkPolyData*);

vtkSmartPointer<vtkPolyData> smooth_mesh(const vtkSmartPointer<vtkPolyData>&, unsigned short);

template <typename T> vtkSmartPointer<vtkFloatArray> scalars_from_image(const vtkSmartPointer<vtkPolyData>& mesh, const typename T::Pointer& img)
{
    const auto points = mesh->GetPoints();
    vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
    scalars->SetNumberOfTuples(mesh->GetNumberOfPoints());
    scalars->SetNumberOfComponents(1);
    for (long long i = 0; i < points->GetNumberOfPoints(); i++) {
        const auto p = points->GetPoint(i);
        auto p_id = img->TransformPhysicalPointToIndex(itk::Point<float, 3>(p));
        auto scalar = img->GetPixel(p_id);
        scalars->SetTuple1(i, scalar);
    }
    return scalars;
}

void compute_normals(vtkSmartPointer<vtkPolyData>&);

int polydata_connected_components(const vtkSmartPointer<vtkPolyData>&);

vtkSmartPointer<vtkPolyData> create_mesh(const std::vector<std::tuple<float, float, float>>&, const std::vector<std::vector<uint32_t>>&);

void center_of_mass(const vtkSmartPointer<vtkPolyData>&, double*);

std::vector<double> mesh_bbox(const vtkSmartPointer<vtkPolyData>&);
