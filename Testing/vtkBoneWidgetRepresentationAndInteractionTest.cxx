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

#include "vtkCallbackCommand.h"
#include <vtkAxes.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkBiDimensionalRepresentation2D.h>
#include <vtkCommand.h>
#include <vtkMath.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAppendPolyData.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>

#include <vtkInteractorStyleTrackballCamera.h>

#include "vtkBoneWidget.h"
#include "vtkCylinderBoneRepresentation.h"
#include "vtkDoubleConeBoneRepresentation.h"
 
// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
 
    virtual void OnKeyPress() 
      {
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();
      std::cout<<"Key pressed: "<<key<<std::endl; 

      if (key == "Control_L")
        {
        int widgetState = this->Widget->GetWidgetState();
   
        if ( widgetState == vtkBoneWidget::Start )
          {
          //do nothing
          //I wanted this if to be explicit !
          }
        else if ( widgetState == vtkBoneWidget::Define )
          {
          //do nothing
          //I wanted this if to be explicit !
          }
        else if ( widgetState == vtkBoneWidget::Rest )
          {
          this->Widget->SetWidgetStateToPose();
          }
        else if ( widgetState == vtkBoneWidget::Pose )
          {
          this->Widget->SetWidgetStateToStart();
          }
        }
      else if (key == "Tab")
        {
        vtkWidgetRepresentation* rep = Widget->GetRepresentation();

        if (vtkCylinderBoneRepresentation::SafeDownCast(rep)) // switch to double cone
            {
            vtkSmartPointer<vtkDoubleConeBoneRepresentation> simsIconRep = 
              vtkSmartPointer<vtkDoubleConeBoneRepresentation>::New();
            simsIconRep->SetNumberOfSides(10);
            simsIconRep->SetRatio(0.2);
            simsIconRep->SetCapping(1);
            simsIconRep->GetConesProperty()->SetOpacity(0.7);
            Widget->SetRepresentation(simsIconRep);
            }
        else if (vtkDoubleConeBoneRepresentation::SafeDownCast(rep)) // switch to line
          {
          vtkSmartPointer<vtkBoneRepresentation> lineRep = 
              vtkSmartPointer<vtkBoneRepresentation>::New();
         
          Widget->SetRepresentation(lineRep);
          }
        else if (vtkBoneRepresentation::SafeDownCast(rep))
          {
          vtkSmartPointer<vtkCylinderBoneRepresentation> cylinderRep = 
            vtkSmartPointer<vtkCylinderBoneRepresentation>::New();
          cylinderRep->SetNumberOfSides(10);
          cylinderRep->GetCylinderProperty()->SetOpacity(0.7);
          Widget->SetRepresentation(cylinderRep);
          }
        }
      }

  vtkBoneWidget* Widget;
};

vtkStandardNewMacro(KeyPressInteractorStyle);

int vtkBoneWidgetRepresentationAndInteractionTest(int, char *[])
{
  // A renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // An interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  vtkSmartPointer<vtkBoneWidget> BoneWidget = 
    vtkSmartPointer<vtkBoneWidget>::New();
  BoneWidget->SetInteractor(renderWindowInteractor);
  //Test Line
  BoneWidget->CreateDefaultRepresentation();

  //Setup callbacks
  vtkSmartPointer<KeyPressInteractorStyle> style = 
    vtkSmartPointer<KeyPressInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  style->Widget = BoneWidget;
  style->SetCurrentRenderer(renderer);
 
  // Render
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  BoneWidget->On();
  //AxesWidget->On();

  // Begin mouse interaction
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}