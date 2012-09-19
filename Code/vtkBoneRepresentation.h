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

//Rip off vtkLineAnnotation

#ifndef __vtkBoneRepresentation_h
#define __vtkBoneRepresentation_h

#include "vtkLineRepresentation.h"
#include "vtkBoneWidgetHeader.h"

class VTK_BONEWIDGETS_EXPORT vtkBoneRepresentation : public vtkLineRepresentation
{
public:
  // Description:
  // Instantiate the class.
  static vtkBoneRepresentation *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkBoneRepresentation,vtkWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Methods to Set/Get the coordinates of the two points defining
  // this representation. Note that methods are available for both
  // display and world coordinates.
  void GetHeadWorldPosition(double pos[3]);
  double* GetHeadWorldPosition();
  void GetHeadDisplayPosition(double pos[3]);
  double* GetHeadDisplayPosition();
  virtual void SetHeadWorldPosition(double pos[3]);
  virtual void SetHeadDisplayPosition(double pos[3]);
  void GetTailDisplayPosition(double pos[3]);
  double* GetTailDisplayPosition();
  void GetTailWorldPosition(double pos[3]);
  double* GetTailWorldPosition();
  virtual void SetTailWorldPosition(double pos[3]);
  virtual void SetTailDisplayPosition(double pos[3]);

  virtual void Highlight(int highlight)
    {
    this->HighlightLine(highlight);
    this->HighlightPoint(0, highlight);
    this->HighlightPoint(1, highlight);
    }

protected:
  vtkBoneRepresentation();
  ~vtkBoneRepresentation();

private:
  vtkBoneRepresentation(const vtkBoneRepresentation&);  //Not implemented
  void operator=(const vtkBoneRepresentation&);  //Not implemented
};

#endif
