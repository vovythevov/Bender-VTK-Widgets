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

#ifndef __vtkCylinderBoneRepresentation_h
#define __vtkCylinderBoneRepresentation_h

#include "vtkBoneRepresentation.h"
#include "vtkBoneWidgetHeader.h"

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;
class vtkProperty;
class vtkTubeFilter;

class VTK_BONEWIDGETS_EXPORT vtkCylinderBoneRepresentation : public vtkBoneRepresentation
{
public:

  // Description:
  // Instantiate this class.
  static vtkCylinderBoneRepresentation *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkCylinderBoneRepresentation, vtkBoneRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Methods to Set/Get the coordinates of the two points defining
  // this representation. Note that methods are available for both
  // display and world coordinates.
  virtual void SetPoint1WorldPosition(double pos[3]);
  virtual void SetPoint2WorldPosition(double pos[3]);

  // Description:
  virtual void GetPolyData(vtkPolyData *pd);

  // Description:
  // These are methods that satisfy vtkWidgetRepresentation's API.
  virtual void BuildRepresentation();
  virtual double *GetBounds();

  vtkGetMacro(Radius,double);

  // Description:
  void SetCapping (int capping);
  vtkGetMacro(Capping, int);

  // Description:
  // Minimum 3
  void SetNumberOfSides(int numberOFSides);
  vtkGetMacro(NumberOfSides, int);

  // Description:
  // Get the cylinder properties. The properties of the cylinder when selected
  // and unselected can be manipulated.
  vtkGetObjectMacro(CylinderProperty,vtkProperty);
  //vtkGetObjectMacro(SelectedCylinderProperty,vtkProperty);

  // Description:
  // Methods supporting the rendering process.
  virtual void GetActors(vtkPropCollection *pc);
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();
  
protected:
  vtkCylinderBoneRepresentation();
  ~vtkCylinderBoneRepresentation();

  // the Cylinder
  vtkActor*          CylinderActor;
  vtkPolyDataMapper* CylinderMapper;
  vtkTubeFilter*     CylinderGenerator;

  // Properties used to control the appearance of selected objects and
  // the manipulator in general.
  vtkProperty* CylinderProperty;
  void         CreateDefaultProperties();

  //Cylinder properties
  double Radius;
  int    Capping;
  int    NumberOfSides;

  void RebuildCylinder();

private:
  vtkCylinderBoneRepresentation(const vtkCylinderBoneRepresentation&);  //Not implemented
  void operator=(const vtkCylinderBoneRepresentation&);  //Not implemented
};

#endif
