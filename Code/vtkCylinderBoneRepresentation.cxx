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
#include "vtkCylinderBoneRepresentation.h"

#include "vtkActor.h"
#include "vtkAppendPolyData.h"
#include "vtkBox.h"
#include "vtkCamera.h"
#include "vtkCylinderSource.h"
#include "vtkCallbackCommand.h"
#include "vtkInteractorObserver.h"
#include "vtkLineSource.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkTransformPolydataFilter.h"
#include "vtkTubeFilter.h"
#include "vtkWindow.h"

vtkStandardNewMacro(vtkCylinderBoneRepresentation);

//----------------------------------------------------------------------------
vtkCylinderBoneRepresentation::vtkCylinderBoneRepresentation()
{
  this->InstantiateHandleRepresentation();

  // Represent the Cylinder
  this->CylinderGenerator= vtkTubeFilter::New();
  this->CylinderMapper = vtkPolyDataMapper::New();
  this->CylinderMapper->SetInput(this->CylinderGenerator->GetOutput());
  this->CylinderActor = vtkActor::New();
  this->CylinderActor->SetMapper(this->CylinderMapper);

  // Set up the initial properties
  this->CreateDefaultProperties();

  this->CylinderActor->SetProperty(this->CylinderProperty);

  this->BuildRepresentation();
}

//----------------------------------------------------------------------------
vtkCylinderBoneRepresentation::~vtkCylinderBoneRepresentation()
{  
  this->CylinderGenerator->Delete();
  this->CylinderActor->Delete();
  this->CylinderMapper->Delete();
}

//-- Set/Get position of the three handles -----------------------------
//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::SetPoint1WorldPosition(double x[3])
{
  this->Superclass::SetPoint1WorldPosition(x);
  this->RebuildCylinder();
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::SetPoint2WorldPosition(double x[3])
{
  this->Superclass::SetPoint2WorldPosition(x);
  this->RebuildCylinder();
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::SetCapping(int capping)
{
  if (this->Capping != capping)
    {
    this->Capping = capping;
    this->RebuildCylinder();
    }
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::SetNumberOfSides(int numberOfSides)
{
  if (numberOfSides >= 3 && this->NumberOfSides != numberOfSides)
    {
    this->NumberOfSides = numberOfSides;
    this->RebuildCylinder();
    }
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::GetPolyData(vtkPolyData *pd)
{
  vtkSmartPointer<vtkAppendPolyData> append =
    vtkSmartPointer<vtkAppendPolyData>::New();

  vtkPolyData* superclassPd = 0;
  this->Superclass::GetPolyData(superclassPd);
  
  append->AddInput( this->CylinderGenerator->GetOutput() );
  append->AddInput( superclassPd );
  append->Update();
  pd->ShallowCopy( append->GetOutput() );
}

//----------------------------------------------------------------------
double *vtkCylinderBoneRepresentation::GetBounds()
{
  this->BuildRepresentation();
  this->BoundingBox->SetBounds(this->CylinderActor->GetBounds());
  return this->BoundingBox->GetBounds();
}

//----------------------------------------------------------------------------
void vtkCylinderBoneRepresentation::CreateDefaultProperties()
{
  this->Superclass::CreateDefaultProperties();
  
  // Cylinder properties
  this->CylinderProperty = vtkProperty::New();
  this->CylinderProperty->SetAmbient(1.0);
  this->CylinderProperty->SetAmbientColor(1.0,1.0,1.0);
  //this->CylinderProperty->SetOpacity(0.3);

  /*this->SelectedCylinderProperty = vtkProperty::New();
  this->SelectedCylinderProperty->SetAmbient(1.0);
  this->SelectedCylinderProperty->SetAmbientColor(0.0,1.0,0.0);
  this->SelectedCylinderProperty->SetOpacity(0.3);*/

  this->Capping = 1;
}

//----------------------------------------------------------------------------
void vtkCylinderBoneRepresentation::BuildRepresentation()
{
  // Rebuild only if necessary
  if ( this->GetMTime() > this->BuildTime ||
       (this->Renderer && this->Renderer->GetVTKWindow() &&
        (this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime ||
        this->Renderer->GetActiveCamera()->GetMTime() > this->BuildTime)) )
    {
    this->Superclass::BuildRepresentation();
    this->RebuildCylinder();   

    this->BuildTime.Modified();
    }
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::RebuildCylinder()
{
  this->CylinderGenerator->SetCapping( this->Capping );
  this->CylinderGenerator->SetRadius( this->Distance / 10 );
  this->CylinderGenerator->SetInput( this->LineSource->GetOutput() );
  this->CylinderGenerator->SetNumberOfSides( this->NumberOfSides );
  this->CylinderGenerator->Update();
}

//----------------------------------------------------------------------
void vtkCylinderBoneRepresentation::GetActors(vtkPropCollection *pc)
{
  this->Superclass::GetActors(pc);
  this->CylinderActor->GetActors(pc);
}

//----------------------------------------------------------------------------
void vtkCylinderBoneRepresentation::ReleaseGraphicsResources(vtkWindow *w)
{
  this->Superclass::ReleaseGraphicsResources(w);
  this->CylinderActor->ReleaseGraphicsResources(w);
}

//----------------------------------------------------------------------------
int vtkCylinderBoneRepresentation::RenderOpaqueGeometry(vtkViewport *v)
{
  this->BuildRepresentation();

  return  this->CylinderActor->RenderOpaqueGeometry(v)
          + this->Superclass::RenderOpaqueGeometry(v);
}

//----------------------------------------------------------------------------
int vtkCylinderBoneRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  this->BuildRepresentation();

  return  this->CylinderActor->RenderTranslucentPolygonalGeometry(v)
          + this->Superclass::RenderTranslucentPolygonalGeometry(v);
}

//----------------------------------------------------------------------------
int vtkCylinderBoneRepresentation::HasTranslucentPolygonalGeometry()
{
  this->BuildRepresentation();

  return  this->CylinderActor->HasTranslucentPolygonalGeometry()
          || this->Superclass::HasTranslucentPolygonalGeometry();
}

//----------------------------------------------------------------------------
void vtkCylinderBoneRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if ( this->CylinderProperty )
    {
    os << indent << "Cylinder Property: " << this->CylinderProperty << "\n";
    }
  else
    {
    os << indent << "Cylinder Property: (none)\n";
    }
  if ( this->SelectedCylinderProperty )
    {
    os << indent << "Selected Cylinder Property: "
       << this->SelectedCylinderProperty << "\n";
    }
  else
    {
    os << indent << "Selected Cylinder Property: (none)\n";
    }

  os << indent << "Number Of Sides: " << this->NumberOfSides << "\n";
  os << indent << "Capping: " << this->Capping << "\n";
  os << indent << "Radius: " << this->Radius << "\n";
}



