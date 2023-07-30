#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkImageData.h>

vtkSmartPointer<vtkPolyData> marching_cubes(const vtkSmartPointer<vtkImageData>&);

vtkSmartPointer<vtkPolyData> simplify_mesh(const vtkSmartPointer<vtkPolyData>&, float);

vtkSmartPointer<vtkPolyData> extract_largest(vtkPolyData*);

vtkSmartPointer<vtkPolyData> smooth_mesh(const vtkSmartPointer<vtkPolyData>&, unsigned short);

void compute_normals(vtkSmartPointer<vtkPolyData>&);

int polydata_connected_components(const vtkSmartPointer<vtkPolyData>&);

vtkSmartPointer<vtkPolyData> create_mesh(const std::vector<std::tuple<float, float, float>>&, const std::vector<std::vector<uint32_t>>&);

void center_of_mass(const vtkSmartPointer<vtkPolyData>&, double*);

std::vector<double> mesh_bbox(const vtkSmartPointer<vtkPolyData>&);
