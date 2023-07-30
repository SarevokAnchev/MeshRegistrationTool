//
// Created by louis on 23/05/2021.
//

#pragma once

#include <filesystem>

#include "vtkSmartPointer.h"
#include "vtkImageData.h"

bool is_dicom_folder(const std::string& path);

bool is_nii_image(const std::string& path);
