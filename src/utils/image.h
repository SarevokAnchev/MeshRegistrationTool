//
// Created by louis on 23/05/2021.
//

#pragma once

#include <filesystem>

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "vtkSmartPointer.h"
#include "vtkImageData.h"
#include "itkImageToVTKImageFilter.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"

template<typename T> typename T::Pointer read_nii(const std::string& path)
{
    auto reader = itk::ImageFileReader<T>::New();
    reader->SetFileName(path);
    reader->Update();
    return reader->GetOutput();
}

template<typename T> void write_nii(const typename T::Pointer& image, const std::string& path)
{
    auto writer = itk::ImageFileWriter<T>::New();
    writer->SetFileName(path);
    writer->SetInput(image);
    writer->Update();
}

template<typename T> vtkSmartPointer<vtkImageData> itk_to_vtk_image(const typename T::Pointer& img)
{
    typename itk::ImageToVTKImageFilter<T>::Pointer filter = itk::ImageToVTKImageFilter<T>::New();
    filter->SetInput(img);
    filter->Update();
    return filter->GetOutput();
}

bool is_dicom_folder(const std::string& path);

bool is_nii_image(const std::string& path);
