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

    QLabel* moving_label;
    QPushButton* moving_button;

    QPushButton* auto_button;
    QPushButton* manual_button;
    QPushButton* save_button;

public:
    explicit MainWindow(QWidget* parent=nullptr);
    static vtkSmartPointer<vtkPolyData> read_mesh_file(const std::filesystem::path&);

public slots:
    void choose_fixed_file();
    void choose_moving_file();

    void auto_registration();
    void manual_registration();
};
