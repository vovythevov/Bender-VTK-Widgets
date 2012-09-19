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

#include "vtkDoubleConeBoneRepresentation.h"

#include "vtkActor.h"
#include "vtkAppendPolyData.h"
#include "vtkBox.h"
#include "vtkCamera.h"
#include "vtkConeSource.h"
#include "vtkLineSource.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkTubeFilter.h"
#include "vtkWindow.h"

vtkStandardNewMacro(vtkDoubleConeBoneRepresentation);

//----------------------------------------------------------------------------
vtkDoubleConeBoneRepresentation::vtkDoubleConeBoneRepresentation()
{
  this->InstantiateHandleRepresentation();

  // Represent the Cone
  this->Cone1 = vtkConeSource::New();
  this->Cone2 = vtkConeSource::New();
  this->Ratio = 0.25;
  this->NumberOfSides = 5;
  this->Capping = 1;

  this->GlueFilter = vtkAppendPolyData::New();
  this->GlueFilter->AddInput( this->Cone1->GetOutput() );
  this->GlueFilter->AddInput( this->Cone2->GetOutput() );
  this->GlueFilter->Update();

  this->ConesMapper = vtkPolyDataMapper::New();
  this->ConesMapper->SetInput(this->GlueFilter->GetOutput());
  this->ConesActor = vtkActor::New();
  this->ConesActor->SetMapper(this->ConesMapper);

  // Set up the initial properties
  this->CreateDefaultProperties();

  this->ConesActor->SetProperty(this->ConesProperty);
}

//----------------------------------------------------------------------------
vtkDoubleConeBoneRepresentation::~vtkDoubleConeBoneRepresentation()
{
  if (this->ConesProperty)
    {
    this->ConesProperty->Delete();
    }

  this->Cone1->Delete();
  this->Cone2->Delete();

  this->GlueFilter->Delete();
  this->ConesActor->Delete();
  this->ConesMapper->Delete();
}

//-- Set/Get position of the three handles -----------------------------
//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::SetPoint1WorldPosition(double x[3])
{
  this->Superclass::SetPoint1WorldPosition(x);
  this->RebuildCones();
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::SetPoint2WorldPosition(double x[3])
{
  this->Superclass::SetPoint2WorldPosition(x);
  this->RebuildCones();
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::SetRatio(double ratio)
{
  if (ratio > 0.0001 && ratio <= 1.0 && this->Ratio != ratio)
    {
    this->Ratio = ratio;
    
    this->RebuildCones();
    this->GlueFilter->Update();
    }
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::SetCapping(int capping)
{
  if (this->Capping != capping)
    {
    this->Capping = capping;
    
    this->Cone1->SetCapping(capping);
    this->Cone2->SetCapping(capping);
    this->GlueFilter->Update();
    }
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::SetNumberOfSides(int numberOfSides)
{
  if (numberOfSides >= 3 && this->NumberOfSides != numberOfSides)
    {
    this->NumberOfSides = numberOfSides;
    
    this->Cone1->SetResolution(numberOfSides);
    this->Cone2->SetResolution(numberOfSides);
    this->GlueFilter->Update();
    }
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::GetPolyData(vtkPolyData *pd)
{
  vtkSmartPointer<vtkAppendPolyData> append =
    vtkSmartPointer<vtkAppendPolyData>::New();

  vtkPolyData* superclassPd = 0;
  this->Superclass::GetPolyData(superclassPd);
  
  append->AddInput( this->GlueFilter->GetOutput() );
  append->AddInput( superclassPd );
  append->Update();
  pd->ShallowCopy( append->GetOutput() );
}

//----------------------------------------------------------------------
double *vtkDoubleConeBoneRepresentation::GetBounds()
{
  this->BuildRepresentation();
  this->BoundingBox->SetBounds(this->ConesActor->GetBounds());
  return this->BoundingBox->GetBounds();
}

//----------------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::CreateDefaultProperties()
{
  // Cones properties
  this->ConesProperty = vtkProperty::New();
  this->ConesProperty->SetAmbient(1.0);
  this->ConesProperty->SetAmbientColor(1.0,1.0,1.0);
  //this->ConesProperty->SetOpacity(0.3);
}

//----------------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::BuildRepresentation()
{
  // Rebuild only if necessary
  if ( this->GetMTime() > this->BuildTime ||
       (this->Renderer && this->Renderer->GetVTKWindow() &&
        (this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime ||
        this->Renderer->GetActiveCamera()->GetMTime() > this->BuildTime)) )
    {
    this->Superclass::BuildRepresentation();
    this->RebuildCones();

    this->BuildTime.Modified();
    }
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::RebuildCones()
{
  double x1[3], x2[3], vect[3];
  this->GetPoint1WorldPosition(x1);
  this->GetPoint2WorldPosition(x2);
  vect[0] = x2[0] - x1[0];
  vect[1] = x2[1] - x1[1];
  vect[2] = x2[2] - x1[2];

  double cone1Ratio = this->Ratio * 0.5;
  double cone2Ratio = (1 + this->Ratio) * 0.5;
  this->Cone1->SetCenter( x1[0] + (vect[0] * cone1Ratio),
                          x1[1] + (vect[1] * cone1Ratio),
                          x1[2] + (vect[2] * cone1Ratio) );
  this->Cone1->SetDirection(-vect[0], -vect[1], -vect[2]);
  this->Cone1->SetHeight( this->Distance * this->Ratio );
  this->Cone1->SetRadius( this->Distance / 10 );

  this->Cone2->SetCenter( x1[0] + (vect[0] * cone2Ratio),
                          x1[1] + (vect[1] * cone2Ratio),
                          x1[2] + (vect[2] * cone2Ratio) );

  this->Cone2->SetDirection(vect);
  this->Cone2->SetHeight( this->Distance * (1 - this->Ratio));
  this->Cone2->SetRadius( this->Distance / 10 );

  this->GlueFilter->Update();
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::GlueCones()
{
  this->GlueFilter->AddInput( this->Cone1->GetOutput() );
  this->GlueFilter->AddInput( this->Cone2->GetOutput() );
  this->GlueFilter->Update();
}

//----------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::GetActors(vtkPropCollection *pc)
{
  this->Superclass::GetActors(pc);
  this->ConesActor->GetActors(pc);
}

//----------------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::ReleaseGraphicsResources(vtkWindow *w)
{
  this->Superclass::ReleaseGraphicsResources(w);
  this->ConesActor->ReleaseGraphicsResources(w);
}

//----------------------------------------------------------------------------
int vtkDoubleConeBoneRepresentation::RenderOpaqueGeometry(vtkViewport *v)
{
  this->BuildRepresentation();

  return  this->ConesActor->RenderOpaqueGeometry(v)
          + this->Superclass::RenderOpaqueGeometry(v);
}

//----------------------------------------------------------------------------
int vtkDoubleConeBoneRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  this->BuildRepresentation();

  return  this->ConesActor->RenderTranslucentPolygonalGeometry(v)
          + this->Superclass::RenderTranslucentPolygonalGeometry(v);
}

//----------------------------------------------------------------------------
int vtkDoubleConeBoneRepresentation::HasTranslucentPolygonalGeometry()
{
  this->BuildRepresentation();

  return  this->ConesActor->HasTranslucentPolygonalGeometry()
          || this->Superclass::HasTranslucentPolygonalGeometry();
}

//----------------------------------------------------------------------------
void vtkDoubleConeBoneRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if ( this->ConesProperty )
    {
    os << indent << "Cone Property: " << this->ConesProperty << "\n";
    }
  else
    {
    os << indent << "Cone Property: (none)\n";
    }

  os << indent << "Number Of Sides: " << this->NumberOfSides << "\n";
  os << indent << "Radius: " << this->Radius << "\n";
}



