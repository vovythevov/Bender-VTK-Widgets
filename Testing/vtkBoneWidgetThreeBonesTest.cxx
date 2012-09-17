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
#include <vtkBoxWidget.h>
#include <vtkBiDimensionalRepresentation2D.h>
#include <vtkCommand.h>
#include <vtkMath.h>
#include <vtkMatrix3x3.h>
#include <vtkTransform.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkLineWidget2.h>
#include <vtkLineRepresentation.h>

#include <vtkInteractorStyleTrackballCamera.h>

#include "vtkBoneWidget.h"
#include "vtkCylinderBoneRepresentation.h"
#include "vtkDoubleConeBoneRepresentation.h"
 
// Define interaction style
class ThreeBonesTestKeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static ThreeBonesTestKeyPressInteractorStyle* New();
    vtkTypeMacro(ThreeBonesTestKeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
 
    virtual void OnKeyPress() 
      {
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();
      std::cout<<"Key Pressed: "<<key<<std::endl;

      if (key == "Control_L")
        {
        int widgetState = this->Widget->GetWidgetState();

        if ( widgetState == vtkBoneWidget::Rest )
          {
          std::cout<<"Set Widget State to Pose"<<std::endl;
          this->Widget->SetWidgetStateToPose();
          this->MiddleSonWidget->SetWidgetStateToPose();
          this->SonWidget->SetWidgetStateToPose();
          }
        else if ( widgetState == vtkBoneWidget::Pose )
          {
          std::cout<<"Set Widget State to Rest"<<std::endl;
          this->Widget->SetWidgetStateToRest();
          this->MiddleSonWidget->SetWidgetStateToRest();
          this->SonWidget->SetWidgetStateToRest();
          }

        }
    else if (key == "r")
        {
        vtkWidgetRepresentation* rep = Widget->GetRepresentation();

        if (vtkCylinderBoneRepresentation::SafeDownCast(rep)) // switch to double cone
            {
            vtkSmartPointer<vtkDoubleConeBoneRepresentation> simsIconRep = 
              vtkSmartPointer<vtkDoubleConeBoneRepresentation>::New();
            simsIconRep->SetNumberOfSides(4);
            simsIconRep->SetRatio(0.2);
            simsIconRep->SetCapping(1);
            simsIconRep->GetConesProperty()->SetOpacity(0.7);
            Widget->SetRepresentation(simsIconRep);
            Widget->GetvtkBoneRepresentation()->GetLineProperty()->SetColor(0.0, 0.0, 1.0);

           vtkSmartPointer<vtkDoubleConeBoneRepresentation> middleSonSimsIconRep = 
              vtkSmartPointer<vtkDoubleConeBoneRepresentation>::New();
            middleSonSimsIconRep->SetNumberOfSides(4);
            middleSonSimsIconRep->SetRatio(0.2);
            middleSonSimsIconRep->SetCapping(1);
            middleSonSimsIconRep->GetConesProperty()->SetOpacity(0.7);
            MiddleSonWidget->SetRepresentation(middleSonSimsIconRep);

            vtkSmartPointer<vtkDoubleConeBoneRepresentation> sonSimsIconRep = 
              vtkSmartPointer<vtkDoubleConeBoneRepresentation>::New();
            sonSimsIconRep->SetNumberOfSides(4);
            sonSimsIconRep->SetRatio(0.2);
            sonSimsIconRep->SetCapping(1);
            sonSimsIconRep->GetConesProperty()->SetOpacity(0.7);
            SonWidget->SetRepresentation(sonSimsIconRep);
            }
        else if (vtkDoubleConeBoneRepresentation::SafeDownCast(rep)) // switch to line
          {
          vtkSmartPointer<vtkBoneRepresentation> lineRep = 
              vtkSmartPointer<vtkBoneRepresentation>::New();
          Widget->SetRepresentation(lineRep);
          Widget->GetvtkBoneRepresentation()->GetLineProperty()->SetColor(0.0, 0.0, 1.0);

          vtkSmartPointer<vtkBoneRepresentation> middleSonLineRep = 
              vtkSmartPointer<vtkBoneRepresentation>::New();
          MiddleSonWidget->SetRepresentation(middleSonLineRep);

          vtkSmartPointer<vtkBoneRepresentation> sonLineRep = 
              vtkSmartPointer<vtkBoneRepresentation>::New();
          SonWidget->SetRepresentation(sonLineRep);
          }
        else if (vtkBoneRepresentation::SafeDownCast(rep))
          {
          vtkSmartPointer<vtkCylinderBoneRepresentation> cylinderRep = 
            vtkSmartPointer<vtkCylinderBoneRepresentation>::New();
          cylinderRep->SetNumberOfSides(10);
          cylinderRep->GetCylinderProperty()->SetOpacity(0.7);
          Widget->SetRepresentation(cylinderRep);
          Widget->GetvtkBoneRepresentation()->GetLineProperty()->SetColor(0.0, 0.0, 1.0);

          vtkSmartPointer<vtkCylinderBoneRepresentation> middleSonCylinderRep = 
            vtkSmartPointer<vtkCylinderBoneRepresentation>::New();
          middleSonCylinderRep->SetNumberOfSides(10);
          middleSonCylinderRep->GetCylinderProperty()->SetOpacity(0.7);
          MiddleSonWidget->SetRepresentation(middleSonCylinderRep);

          vtkSmartPointer<vtkCylinderBoneRepresentation> sonCylinderRep = 
            vtkSmartPointer<vtkCylinderBoneRepresentation>::New();
          sonCylinderRep->SetNumberOfSides(10);
          sonCylinderRep->GetCylinderProperty()->SetOpacity(0.7);
          SonWidget->SetRepresentation(sonCylinderRep);
          }
        }
      else if (key == "Tab")
        {
        //std::cout<<"Tab"<<std::endl;
        int state = Widget->GetDebugAxes() + 1;
        if (state > vtkBoneWidget::ShowPoseTransformAndOrientation)
          {
          state = 0;
          }
        Widget->SetDebugAxes(state);
        SonWidget->SetDebugAxes(state);
        MiddleSonWidget->SetDebugAxes(state);
        }
      else if (key == "l")
        {
        MiddleSonWidget->SetP1LinkedToParent(
         ! MiddleSonWidget->GetP1LinkedToParent() );
        SonWidget->SetP1LinkedToParent(
         ! SonWidget->GetP1LinkedToParent() );
        }
      }

    vtkBoneWidget* Widget;
    vtkBoneWidget* MiddleSonWidget;
    vtkBoneWidget* SonWidget;
};

vtkStandardNewMacro(ThreeBonesTestKeyPressInteractorStyle);

int vtkBoneWidgetThreeBonesTest(int, char *[])
{
  std::cout<<"Commands: "
    <<"Tab: Switch debug state"<<std::endl
    <<"Control_L: Switch Rest/Pose mode"<<std::endl
    <<"l: link/unlink bone"<<std::endl
    <<"r: change representation"<<std::endl;

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

  // A box to visualize things better
  vtkSmartPointer<vtkBoxWidget> box = vtkSmartPointer<vtkBoxWidget>::New();
  box->SetInteractor(renderWindowInteractor);
  box->SetScalingEnabled(false);
  box->SetRotationEnabled(false);
  box->SetHandleSize(0.01);
  box->SetOutlineCursorWires(false);
  box->GetHandleProperty()->SetLineWidth(0.001);
  box->GetFaceProperty()->SetRepresentationToSurface();
  box->SetTranslationEnabled(false);
  box->PlaceWidget(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

  vtkSmartPointer<vtkBoneWidget> fatherBoneWidget = 
    vtkSmartPointer<vtkBoneWidget>::New();
  fatherBoneWidget->SetInteractor(renderWindowInteractor);
  fatherBoneWidget->SetCurrentRenderer(renderer);
  fatherBoneWidget->CreateDefaultRepresentation();

  fatherBoneWidget->GetvtkBoneRepresentation()->GetLineProperty()->SetColor(1.0, 0.0, 0.0);
  fatherBoneWidget->GetvtkBoneRepresentation()->GetPoint1Representation()->GetProperty()->SetColor(0.0, 1.0, 1.0);
  fatherBoneWidget->GetvtkBoneRepresentation()->GetPoint2Representation()->GetProperty()->SetColor(0.0, 0.0, 1.0);
  fatherBoneWidget->SetWidgetStateToRest();

  //Reset Father position
  fatherBoneWidget->SetPoint1WorldPosition(0.0, 0.0, 0.0);
  fatherBoneWidget->SetPoint2WorldPosition(0.0, 0.1, 0.0);

  vtkSmartPointer<vtkBoneWidget> middleSonBoneWidget = 
    vtkSmartPointer<vtkBoneWidget>::New();
  middleSonBoneWidget->SetInteractor(renderWindowInteractor);
  middleSonBoneWidget->CreateDefaultRepresentation();

  middleSonBoneWidget->SetWidgetStateToRest();
  middleSonBoneWidget->SetPoint1WorldPosition(0.0, 0.2, 0.0);
  middleSonBoneWidget->SetPoint2WorldPosition(0.1, 0.2, 0.0);
  middleSonBoneWidget->SetBoneParent(fatherBoneWidget);
  middleSonBoneWidget->GetvtkBoneRepresentation()->GetPoint1Representation()->GetProperty()->SetColor(0.0, 1.0, 1.0);
  middleSonBoneWidget->GetvtkBoneRepresentation()->GetPoint2Representation()->GetProperty()->SetColor(0.0, 0.0, 1.0);

  vtkSmartPointer<vtkBoneWidget> sonBoneWidget = 
    vtkSmartPointer<vtkBoneWidget>::New();
  sonBoneWidget->SetInteractor(renderWindowInteractor);
  sonBoneWidget->CreateDefaultRepresentation();

  sonBoneWidget->SetWidgetStateToRest();
  sonBoneWidget->SetPoint1WorldPosition(0.1, 0.3, 0.0);
  sonBoneWidget->SetPoint2WorldPosition(0.1, 0.4, 0.0);
  sonBoneWidget->SetBoneParent(middleSonBoneWidget);
  sonBoneWidget->GetvtkBoneRepresentation()->GetPoint1Representation()->GetProperty()->SetColor(0.0, 1.0, 1.0);
  sonBoneWidget->GetvtkBoneRepresentation()->GetPoint2Representation()->GetProperty()->SetColor(0.0, 0.0, 1.0);

  //Setup callbacks
  vtkSmartPointer<ThreeBonesTestKeyPressInteractorStyle> style = 
    vtkSmartPointer<ThreeBonesTestKeyPressInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  style->Widget = fatherBoneWidget;
  style->MiddleSonWidget = middleSonBoneWidget;
  style->SonWidget = sonBoneWidget;
  style->SetCurrentRenderer(renderer);

  vtkSmartPointer<vtkAxesActor> axes = 
    vtkSmartPointer<vtkAxesActor>::New();
 
  vtkSmartPointer<vtkOrientationMarkerWidget> axesWidget = 
    vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  axesWidget->SetOrientationMarker( axes );
  axesWidget->SetInteractor( renderWindowInteractor );
  axesWidget->On();


  // Render
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  fatherBoneWidget->On();
  middleSonBoneWidget->On();
  sonBoneWidget->On();

  //box->On();

  // Begin mouse interaction
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}