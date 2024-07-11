//
// Created by louis on 30/07/23.
//

#pragma once

#include <QApplication>
#include <filesystem>

#include <QWidget>
#include <QLabel>
#include <QPushButton>

#include <vtkPolyData.h>

#include <Eigen/Dense>

#include "../registration/MeshRegistration.h"

class MainWindow : public QWidget {
Q_OBJECT

private:
    MeshRegistration registration;

    QLabel* fixed_label;
    QPushButton* fixed_button;
    QPushButton* save_fixed_button;

    QLabel* moving_label;
    QPushButton* moving_button;
    QPushButton* save_moving_button;

    QPushButton* auto_button;
    QPushButton* manual_button;
    QPushButton* load_button;
    QPushButton* save_button;
    QPushButton* save_mesh_button;

public:
    explicit MainWindow(QWidget* parent=nullptr);
    static vtkSmartPointer<vtkPolyData> read_mesh_file(const std::filesystem::path&);

public slots:
    void choose_fixed_file();
    void save_fixed_mesh();
    void choose_moving_file();
    void save_moving_mesh();

    void auto_registration();
    void manual_registration();

    void load_transform();
    void save_transform();
    void save_moved_mesh();
};
