#include "mesh.h"

#include "vtkImageMarchingCubes.h"
#include "vtkQuadricDecimation.h"
#include "vtkPolyDataConnectivityFilter.h"
#include "vtkWindowedSincPolyDataFilter.h"
#include "vtkCenterOfMass.h"
#include "vtkIdTypeArray.h"
#include "vtkPointData.h"

float nodes_distance(const std::tuple<float, float, float>& a, const std::tuple<float, float, float>& b)
{
    return sqrt((std::get<0>(b)-std::get<0>(a)) * (std::get<0>(b)-std::get<0>(a))
                + (std::get<1>(b)-std::get<1>(a)) * (std::get<1>(b)-std::get<1>(a))
                + (std::get<2>(b)-std::get<2>(a)) * (std::get<2>(b)-std::get<2>(a)));
}

vtkSmartPointer<vtkPolyData> marching_cubes(const vtkSmartPointer<vtkImageData>& img)
{
    vtkSmartPointer<vtkImageMarchingCubes> mc = vtkSmartPointer<vtkImageMarchingCubes>::New();
    mc->SetInputData(img);
    mc->ComputeNormalsOn();
    mc->ComputeGradientsOn();
    mc->SetValue(0, 0.5);
    mc->Update();
    return mc->GetOutput();
}

vtkSmartPointer<vtkPolyData> simplify_mesh(const vtkSmartPointer<vtkPolyData>& mesh, float ratio)
{
    vtkSmartPointer<vtkQuadricDecimation> filter = vtkSmartPointer<vtkQuadricDecimation>::New();
    filter->SetInputData(mesh);
    filter->SetTargetReduction(ratio);
    filter->Update();
    vtkSmartPointer<vtkPolyData> ret_mesh = filter->GetOutput();
    return ret_mesh;
}

vtkSmartPointer<vtkPolyData> extract_largest(vtkPolyData* mesh)
{
    vtkSmartPointer<vtkPolyDataConnectivityFilter> filter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    filter->SetInputData(mesh);
    filter->SetExtractionModeToLargestRegion();
    filter->Update();
    auto region = filter->GetOutput();
    return region;
}

vtkSmartPointer<vtkPolyData> smooth_mesh(const vtkSmartPointer<vtkPolyData>& mesh, unsigned short n_iter)
{
    vtkSmartPointer<vtkWindowedSincPolyDataFilter> filter = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
    filter->SetInputData(mesh);
    filter->SetNumberOfIterations(n_iter);
    filter->BoundarySmoothingOff();
    filter->FeatureEdgeSmoothingOff();
    filter->SetFeatureAngle(120.);
    filter->NonManifoldSmoothingOn();
    filter->NormalizeCoordinatesOn();
    filter->Update();
    return filter->GetOutput();
}

void compute_normals(vtkSmartPointer<vtkPolyData>& mesh)
{
    vtkSmartPointer<vtkPolyDataNormals> filter = vtkSmartPointer<vtkPolyDataNormals>::New();
    filter->SetInputData(mesh);
    filter->ComputePointNormalsOn();
    filter->SetSplitting(false);
    filter->SetConsistency(false);
    filter->Update();
    mesh = filter->GetOutput();
}

int polydata_connected_components(const vtkSmartPointer<vtkPolyData>& mesh)
{
    vtkSmartPointer<vtkPolyDataConnectivityFilter> filter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    filter->SetInputData(mesh);
    filter->SetExtractionModeToAllRegions();
    filter->Update();
    return filter->GetNumberOfExtractedRegions();
}

vtkSmartPointer<vtkPolyData> create_mesh(const std::vector<std::tuple<float, float, float>>& mesh_vertices, const std::vector<std::vector<uint32_t>>& mesh_polygons)
{
    vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> vertices = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();

    vertices->SetNumberOfPoints((long long)mesh_vertices.size());
    for (unsigned int i = 0; i < mesh_vertices.size(); i++) {
        float vert[] = {
            std::get<0>(mesh_vertices[i]),
            std::get<1>(mesh_vertices[i]),
            std::get<2>(mesh_vertices[i])
        };
        vertices->SetPoint(i, vert);
    }
    polygons->SetNumberOfCells((long long)mesh_polygons.size());
    for (auto poly_it = mesh_polygons.begin(); poly_it < mesh_polygons.end(); poly_it++) {
        vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
        for (auto it = poly_it->begin(); it < poly_it->end(); it++) {
            id_list->InsertNextId(*it);
        }
        polygons->InsertNextCell(id_list);
    }
    mesh->SetPoints(vertices);
    mesh->SetPolys(polygons);
    return mesh;
}

void center_of_mass(const vtkSmartPointer<vtkPolyData>& mesh, double* center)
{
    auto filter = vtkSmartPointer<vtkCenterOfMass>::New();
    filter->SetInputData(mesh);
    filter->Update();
    filter->GetCenter(center);
}

std::vector<double> mesh_bbox(const vtkSmartPointer<vtkPolyData>& mesh)
{
    auto* points = mesh->GetPoints();
    auto n_points = mesh->GetNumberOfPoints();
    auto min_x = std::numeric_limits<double>::infinity();
    auto max_x = -std::numeric_limits<double>::infinity();
    auto min_y = std::numeric_limits<double>::infinity();
    auto max_y = -std::numeric_limits<double>::infinity();
    auto min_z = std::numeric_limits<double>::infinity();
    auto max_z = -std::numeric_limits<double>::infinity();
    double p[3];
    for (int i = 0; i < n_points; i++) {
        points->GetPoint(i, p);
        if (p[0] < min_x) min_x = p[0];
        if (p[0] > max_x) max_x = p[0];
        if (p[1] < min_y) min_y = p[1];
        if (p[1] > max_y) max_y = p[1];
        if (p[2] < min_z) min_z = p[2];
        if (p[2] > max_z) max_z = p[2];
    }
    return {
        min_x, max_x,
        min_y, max_y,
        min_z, max_z
    };
}
