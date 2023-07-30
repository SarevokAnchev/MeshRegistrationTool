//
// Created by louis on 23/05/2021.
//

#include "image.h"

bool is_dicom_folder(const std::string& path)
{
    std::filesystem::path folder(path);

    if (!std::filesystem::is_directory(folder)) return false;

    std::vector<std::string> files;
    for (const auto& f : std::filesystem::directory_iterator(folder)) {
        if (f.path().extension().string() == ".dcm" || f.path().extension().string() == ".DCM") {
            return true;
        }
    }
    return false;
}

bool is_nii_image(const std::string& path)
{
    std::filesystem::path f(path);

    if (std::filesystem::is_directory(f)) return false;

    auto extension = f.extension().string();
    if (extension == ".nii"
    || extension == ".gz" && f.stem().extension().string() == ".nii")
        return true;
    return false;
}
