//
// Created by louis on 30/07/23.
//

#include "MainWindow.h"

#include <filesystem>
#include <exception>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>

#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkNIFTIImageReader.h>

#include "../utils/image.h"
#include "../utils/mesh.h"
#include "../registration/ICP.h"
#include "../registration/MeshRegistrationDisplay.h"

MainWindow::MainWindow(QWidget* parent)
{
    auto main_layout = new QVBoxLayout();

    auto fixed_layout = new QHBoxLayout();
    fixed_label = new QLabel("Fixed Mesh");
    fixed_layout->addWidget(fixed_label);
    fixed_button = new QPushButton("File..");
    fixed_layout->addWidget(fixed_button);
    main_layout->addLayout(fixed_layout);

    auto moving_layout = new QHBoxLayout();
    moving_label = new QLabel("Moving Mesh");
    moving_layout->addWidget(moving_label);
    moving_button = new QPushButton("File..");
    moving_layout->addWidget(moving_button);
    main_layout->addLayout(moving_layout);

    auto reg_layout = new QHBoxLayout();
    auto_button = new QPushButton("Auto");
    reg_layout->addWidget(auto_button);
    manual_button = new QPushButton("Manual");
    reg_layout->addWidget(manual_button);
    main_layout->addLayout(reg_layout);

    save_button = new QPushButton("Save Transform");
    main_layout->addWidget(save_button);

    this->setLayout(main_layout);

    connect(fixed_button, &QPushButton::clicked, this, &MainWindow::choose_fixed_file);
    connect(moving_button, &QPushButton::clicked, this, &MainWindow::choose_moving_file);
    connect(auto_button, &QPushButton::clicked, this, &MainWindow::auto_registration);
    connect(manual_button, &QPushButton::clicked, this, &MainWindow::manual_registration);
}

vtkSmartPointer<vtkPolyData> MainWindow::read_mesh_file(const std::filesystem::path& path)
{
    if (is_nii_image(path)) {
        auto reader = vtkSmartPointer<vtkNIFTIImageReader>::New();
        reader->SetFileName(path.c_str());
        reader->Update();
        auto mesh = marching_cubes(reader->GetOutput());
        if (mesh->GetNumberOfPoints() > 5000) {
            mesh = simplify_mesh(mesh, (float)(1 - 5000/(float)mesh->GetNumberOfPoints()));
        }
        return mesh;
    }
    else {
        auto extension = path.extension().string();
        if (extension == ".vtk") {
            vtkSmartPointer<vtkPolyDataReader> reader = vtkPolyDataReader::New();
            reader->SetFileName(path.string().c_str());
            reader->Update();
            return reader->GetOutput();
        }
        else if (extension == ".stl") {
            vtkSmartPointer<vtkSTLReader> reader = vtkSTLReader::New();
            reader->SetFileName(path.string().c_str());
            reader->Update();
            return reader->GetOutput();
        }
        else {
            throw std::invalid_argument("Invalid file type.");
        }
    }
}

void MainWindow::choose_fixed_file()
{
    auto file = QFileDialog::getOpenFileName(
            this, "Mesh file selection");
    auto path = std::filesystem::path(file.toStdString());
    try {
        registration.set_fixed(read_mesh_file(path));
        registration.center();
        fixed_label->setText(QString::fromStdString(path.filename().string()));
    }
    catch (const std::invalid_argument& e) {
    }
    catch (const std::exception& e) {
    }
}

void MainWindow::choose_moving_file()
{
    auto file = QFileDialog::getOpenFileName(
            this, "Mesh file selection");
    auto path = std::filesystem::path(file.toStdString());
    try {
        registration.set_moving(read_mesh_file(path));
        registration.center();
        moving_label->setText(QString::fromStdString(path.filename().string()));
    }
    catch (const std::invalid_argument& e) {
    }
    catch (const std::exception& e) {
    }
}

void MainWindow::auto_registration()
{
    if (!registration.is_valid()) return;
    auto fixed = registration.get_fixed();
    auto moving = registration.get_moving();

    auto fixed_points = fixed->GetNumberOfPoints();
    auto moving_points = moving->GetNumberOfPoints();
    auto fixed_keep_interval = static_cast<int>((double)fixed_points/500);
    auto moving_keep_interval = static_cast<int>((double)moving_points/500);

    ICP::PointCloudType fixed_array;
    fixed_array.reserve(fixed_points/fixed_keep_interval + 1);
    long long j = 0;
    for (long long i = 0; i < fixed_points; i++) {
        if (i%fixed_keep_interval == 0) {
            const auto p = fixed->GetPoint(i);
            fixed_array.emplace_back(std::vector<double>{p[0], p[1], p[2]});
            j++;
        }
    }
    ICP::PointCloudType moving_array;
    moving_array.reserve(moving_points/moving_keep_interval + 1);
    j = 0;
    for (long long i = 0; i < moving_points; i++) {
        if (i%moving_keep_interval == 0) {
            const auto p = moving->GetPoint(i);
            moving_array.emplace_back(std::vector<double>{p[0], p[1], p[2]});
            j++;
        }
    }
    ICPParams params = {
            true, 150, 0.0001
    };
    auto res = icp_registration(fixed_array, moving_array, params);
    registration.set_transform(res);
}

void MainWindow::manual_registration()
{
    if (!registration.is_valid()) return;
    auto fixed = registration.get_fixed();
    auto moving = registration.get_moving();
    MeshRegistrationDisplay reg_display(fixed, moving);
    reg_display.set_transform(registration.get_transform());
    reg_display.run();
    registration.set_transform(reg_display.get_transform());
}
