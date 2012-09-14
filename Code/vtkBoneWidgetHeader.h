/*=========================================================================

  Program: Bender

  Copyright (c) Kitware Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0.txt

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

=========================================================================*/

#ifndef _vtkBoneWidget_DLLDEFINES_H_
#define _vtkBoneWidget_DLLDEFINES_H_

#include "vtkBoneWidgetConfigure.h"

#if defined(_WIN32) || defined(WIN32)

 #if defined(VTK_BONE_WIDGET_BUILD_SHARED_LIBS)

  #if defined(vtkBoneWidget_EXPORTS)
   #define VTK_BONEWIDGETS_EXPORT __declspec(dllexport)

  #else
   #define VTK_BONEWIDGETS_EXPORT __declspec(dllimport)

  #endif

 #else
  #define VTK_BONEWIDGETS_EXPORT
 #endif

#endif

#endif