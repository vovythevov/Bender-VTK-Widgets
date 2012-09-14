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

#ifndef __vtkDoubleConeBoneRepresentation_h
#define __vtkDoubleConeBoneRepresentation_h

// Cone1's base on point 1 and cone2's base on point2

#include "vtkBoneRepresentation.h"
#include "vtkBoneWidgetHeader.h"

class vtkActor;
class vtkAppendPolyData;
class vtkPolyDataMapper;
class vtkConeSource;
class vtkProperty;
class vtkPolyData;
class vtkTubeFilter;

class VTK_BONEWIDGETS_EXPORT vtkDoubleConeBoneRepresentation : public vtkBoneRepresentation
{
public:

  // Description:
  // Instantiate this class.
  static vtkDoubleConeBoneRepresentation *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkDoubleConeBoneRepresentation, vtkBoneRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Methods to Set/Get the coordinates of the two points defining
  // this representation. Note that methods are available for both
  // display and world coordinates.
  virtual void SetPoint1WorldPosition(double pos[3]);
  virtual void SetPoint2WorldPosition(double pos[3]);

  // Description:
  void GetPolyData(vtkPolyData *pd);

  // Description:
  // These are methods that satisfy vtkWidgetRepresentation's API.
  virtual void BuildRepresentation();
  virtual double *GetBounds();

  vtkGetMacro(Radius,double);

  // Description:
  // Min 0.0001
  void SetCapping (int capping);
  vtkGetMacro(Capping, int);
  
  // Description:
  //Set the shar ratio between the 2 cones. If set to 0,
  // cone1 takes the whole line. If set to 1, Cone2 take the whole line
  void SetRatio (double ratio);
  vtkGetMacro(Ratio,double);

  // Description:
  // Minimum 3
  void SetNumberOfSides(int numberOFSides);
  vtkGetMacro(NumberOfSides, int);

  // Description:
  // Get the cylinder properties. The properties of the cones when selected
  // and unselected can be manipulated.
  vtkGetObjectMacro(ConesProperty,vtkProperty);
  //vtkGetObjectMacro(SelectedCone1Property,vtkProperty);

  // Description:
  // Methods supporting the rendering process.
  virtual void GetActors(vtkPropCollection *pc);
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();
  
protected:
  vtkDoubleConeBoneRepresentation();
  ~vtkDoubleConeBoneRepresentation();

  // the Cylinder
  vtkActor*          ConesActor;
  vtkPolyDataMapper* ConesMapper;
  vtkConeSource*     Cone1;
  vtkConeSource*     Cone2;

  // Properties used to control the appearance of selected objects and
  // the manipulator in general.
  vtkProperty* ConesProperty;
  //vtkProperty* SelectedCylinderProperty;
  void         CreateDefaultProperties();

  // Ivars used during widget interaction to hold initial positions
  double StartCylinderHandle[3];

  //Cone properties
  double Radius;
  int    NumberOfSides;
  double Ratio;
  int    Capping;

  vtkAppendPolyData* GlueFilter;
  void GlueCones();
  void RebuildCones();

private:
  vtkDoubleConeBoneRepresentation(const vtkDoubleConeBoneRepresentation&);  //Not implemented
  void operator=(const vtkDoubleConeBoneRepresentation&);  //Not implemented
};

#endif
