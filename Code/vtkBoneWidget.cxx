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

#include "vtkBoneWidget.h"

//My includes
#include "vtkBoneRepresentation.h"

//VTK Includes
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkHandleRepresentation.h>
#include <vtkHandleWidget.h>
#include <vtkLineRepresentation.h>
#include <vtkLineWidget2.h>
#include <vtkMath.h>
#include <vtkMatrix3x3.h>
#include <vtkObjectFactory.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkWidgetEvent.h>

vtkStandardNewMacro(vtkBoneWidget);

//Static declaration of the world coordinates
static const double X[3] = {1.0, 0.0, 0.0};
static const double Y[3] = {0.0, 1.0, 0.0};
static const double Z[3] = {0.0, 0.0, 1.0};

namespace
{

void MultiplyQuaternion(double* quad1, double* quad2, double resultQuad[4])
{
  //Quaternion are (w, x, y, z)
  //The multiplication is given by :
  //(Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
  //(Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
  //(Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
  //(Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)

  resultQuad[0] = quad1[0]*quad2[0] - quad1[1]*quad2[1]
                  - quad1[2]*quad2[2] - quad1[3]*quad2[3];

  resultQuad[1] = quad1[0]*quad2[1] + quad1[1]*quad2[0]
                  + quad1[2]*quad2[3] - quad1[3]*quad2[2];

  resultQuad[2] = quad1[0]*quad2[2] + quad1[2]*quad2[0]
                  + quad1[3]*quad2[1] - quad1[1]*quad2[3];

  resultQuad[3] = quad1[0]*quad2[3] + quad1[3]*quad2[0]
                  + quad1[1]*quad2[2] - quad1[2]*quad2[1];
}

void NormalizeQuaternion(double* quad)
{
  double mag2 = quad[0]*quad[0] + quad[1]*quad[1] + quad[2]*quad[2] + quad[3]*quad[3];
  double mag = sqrt(mag2);
  quad[0] /= mag;
  quad[1] /= mag;
  quad[2] /= mag;
  quad[3] /= mag;
}

void InitializeQuaternion(double* quad)
{
  quad[0] = 1.0;
  quad[1] = 0.0;
  quad[2] = 0.0;
  quad[3] = 0.0;
}

void CopyQuaternion(double* quad, double* copyQuad)
{
  copyQuad[0] = quad[0];
  copyQuad[1] = quad[1];
  copyQuad[2] = quad[2];
  copyQuad[3] = quad[3];
}

void InitializeVector3(double* vec)
{
  vec[0] = 0.0;
  vec[1] = 0.0;
  vec[2] = 0.0;
}

void CopyVector3(double* vec, double* copyVec)
{
  copyVec[0] = vec[0];
  copyVec[1] = vec[1];
  copyVec[2] = vec[2];
}

}// end namespace


class vtkBoneWidgetCallback : public vtkCommand
{
public:
  static vtkBoneWidgetCallback *New()
    { return new vtkBoneWidgetCallback; }
  virtual void Execute(vtkObject* caller, unsigned long eventId, void*)
    {
      switch (eventId)
        {
        case vtkCommand::StartInteractionEvent:
          {
          this->BoneWidget->StartBoneInteraction();
          break;
          }
        case vtkCommand::EndInteractionEvent:
          {
          this->BoneWidget->EndBoneInteraction();
          break;
          }
        case vtkBoneWidget::RestChangedEvent:
          {
          if (this->BoneWidget->BoneParent ==
                vtkBoneWidget::SafeDownCast(caller))
            {
            this->BoneWidget->BoneParentRestChanged();
            }
          break;
          }
        case vtkBoneWidget::PoseChangedEvent:
          {
          if (this->BoneWidget->BoneParent ==
                vtkBoneWidget::SafeDownCast(caller))
            {
            this->BoneWidget->BoneParentPoseChanged();
            }
          break;
          }
        case vtkBoneWidget::PoseInteractionStoppedEvent:
          {
          if (this->BoneWidget->BoneParent ==
                vtkBoneWidget::SafeDownCast(caller))
            {
            this->BoneWidget->BoneParentInteractionStopped();
            }
          break;
          }
        }
    }
    vtkBoneWidget *BoneWidget;
};

//----------------------------------------------------------------------
vtkBoneWidget::vtkBoneWidget()
{
  this->ManagesCursor = 1;

  this->WidgetState = vtkBoneWidget::Start;
  
  this->BoneParent = NULL;
  InitializeVector3(this->LocalRestP1);
  InitializeVector3(this->LocalRestP2);
  InitializeVector3(this->LocalPoseP1);
  InitializeVector3(this->LocalPoseP2);
  InitializeVector3(this->TemporaryPoseP1);
  InitializeVector3(this->TemporaryPoseP2);

  this->Roll = 0.0;

  InitializeQuaternion(this->Orientation);
  InitializeQuaternion(this->PoseTransform);

  // Manage priorities, we want the handles to be lower priority
  if ( this->Priority <= 0.0 )
    {
    this->Priority = 0.01;
    }

  // The widgets for moving the end points. They observe this widget (i.e.,
  // this widget is the parent to the handles).
  this->Point1Widget = vtkHandleWidget::New();
  this->Point1Widget->SetPriority(this->Priority-0.01);
  this->Point1Widget->SetParent(this);
  this->Point1Widget->ManagesCursorOff();

  this->Point2Widget = vtkHandleWidget::New();
  this->Point2Widget->SetPriority(this->Priority-0.01);
  this->Point2Widget->SetParent(this);
  this->Point2Widget->ManagesCursorOff();

  // Set up the callbacks on the two handles
  this->BoneWidgetCallback1 = vtkBoneWidgetCallback::New();
  this->BoneWidgetCallback1->BoneWidget = this;
  this->Point1Widget->AddObserver(vtkCommand::StartInteractionEvent, this->BoneWidgetCallback1,
                                  this->Priority);
  this->Point1Widget->AddObserver(vtkCommand::EndInteractionEvent, this->BoneWidgetCallback1,
                                  this->Priority);

  this->BoneWidgetCallback2 = vtkBoneWidgetCallback::New();
  this->BoneWidgetCallback2->BoneWidget = this;
  this->Point2Widget->AddObserver(vtkCommand::StartInteractionEvent, this->BoneWidgetCallback2,
                                  this->Priority);
  this->Point2Widget->AddObserver(vtkCommand::EndInteractionEvent, this->BoneWidgetCallback2,
                                  this->Priority);

  // Set up the callbacks for the children
  this->BoneWidgetChildrenCallback = vtkBoneWidgetCallback::New();
  this->BoneWidgetChildrenCallback->BoneWidget = this;

  // These are the event callbacks supported by this widget
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkWidgetEvent::AddPoint,
                                          this, vtkBoneWidget::AddPointAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
                                          vtkWidgetEvent::Move,
                                          this, vtkBoneWidget::MoveAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                          vtkWidgetEvent::EndSelect,
                                          this, vtkBoneWidget::EndSelectAction);

  //Debug axes init
  this->DebugAxes = vtkBoneWidget::Nothing;
  this->DebugX = vtkLineWidget2::New();
  this->DebugY = vtkLineWidget2::New();
  this->DebugZ = vtkLineWidget2::New();
  this->DebugAxesSize = 0.2;

  //Widget interaction init
  this->BoneSelected = 0;
  this->Point1Selected = 0;
  this->Point2Selected = 0;

  //parentage link init
  this->P1LinkedToParent = 0;
  this->ShowParentage = 0;
  this->ParentageLink = vtkLineWidget2::New();

  this->RebuildDebugAxes();
  this->RebuildParentageLink();
}

//----------------------------------------------------------------------
vtkBoneWidget::~vtkBoneWidget()
{
  this->ParentageLink->Delete();

  this->DebugX->Delete();
  this->DebugY->Delete();
  this->DebugZ->Delete();

  this->Point1Widget->RemoveObserver(this->BoneWidgetCallback1);
  this->Point1Widget->Delete();
  this->BoneWidgetCallback1->Delete();

  this->Point2Widget->RemoveObserver(this->BoneWidgetCallback2);
  this->Point2Widget->Delete();
  this->BoneWidgetCallback2->Delete();

  this->BoneWidgetChildrenCallback->Delete();
}

//----------------------------------------------------------------------
void vtkBoneWidget::CreateDefaultRepresentation()
{
  //Init the bone
  if ( ! this->WidgetRep )
    {
    this->WidgetRep = vtkBoneRepresentation::New();
    }

  vtkBoneRepresentation::SafeDownCast(this->WidgetRep)->
    InstantiateHandleRepresentation();

  //Init the debug axes
  this->DebugX->SetInteractor(this->Interactor);
  this->DebugX->GetRepresentation()->SetRenderer(this->CurrentRenderer);
  this->DebugX->CreateDefaultRepresentation();
  this->DebugX->SetEnabled(1);
  vtkLineRepresentation::SafeDownCast(DebugX->GetRepresentation())->SetLineColor(1.0, 0.0, 0.0);
  this->DebugX->SetProcessEvents(0); //So the debug axes aren't interacting
  // vvvv So the axes aren't highlighted vvvv
  vtkLineRepresentation::SafeDownCast(DebugX->GetRepresentation())->SetRepresentationState(0);

  this->DebugY->SetInteractor(this->Interactor);
  this->DebugY->GetRepresentation()->SetRenderer(this->CurrentRenderer);
  this->DebugY->CreateDefaultRepresentation();
  this->DebugY->SetEnabled(1);
  vtkLineRepresentation::SafeDownCast(DebugY->GetRepresentation())->SetLineColor(0.0, 1.0, 0.0);
  this->DebugY->SetProcessEvents(0); //So the debug axes aren't interacting
  // vvvv So the axes aren't highlighted vvvv
  vtkLineRepresentation::SafeDownCast(DebugY->GetRepresentation())->SetRepresentationState(0);

  this->DebugZ->SetInteractor(this->Interactor);
  this->DebugZ->GetRepresentation()->SetRenderer(this->CurrentRenderer);
  this->DebugZ->CreateDefaultRepresentation();
  this->DebugZ->SetEnabled(1);
  vtkLineRepresentation::SafeDownCast(DebugZ->GetRepresentation())->SetLineColor(0.0, 0.0, 1.0);
  this->DebugZ->SetProcessEvents(0); //So the debug axes aren't interacting
  // vvvv So the axes aren't highlighted vvvv
  vtkLineRepresentation::SafeDownCast(DebugZ->GetRepresentation())->SetRepresentationState(0);

  this->ParentageLink->SetInteractor(this->Interactor);
  this->ParentageLink->GetRepresentation()->SetRenderer(this->CurrentRenderer);
  this->ParentageLink->CreateDefaultRepresentation();
  //make dotted line
  vtkLineRepresentation::SafeDownCast(ParentageLink->GetRepresentation())
    ->GetLineProperty()->SetLineStipplePattern(0xf0f0);
  this->ParentageLink->SetEnabled(1);
  this->ParentageLink->SetProcessEvents(0); //So the link isn't interacting
  // vvvv So the link isn't highlighted vvvv
  vtkLineRepresentation::SafeDownCast(ParentageLink->GetRepresentation())->SetRepresentationState(0);

}

//----------------------------------------------------------------------
void vtkBoneWidget::GetPoint1WorldPosition(double p1[3])
{
  CopyVector3(
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(),
    p1);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetPoint1WorldPosition()
{
  return this->GetvtkBoneRepresentation()->GetPoint1WorldPosition();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetPoint1WorldPosition(double x, double y, double z)
{
   double p1[3];
   p1[0] = x;
   p1[1] = y;
   p1[2] = z;
   this->SetPoint1WorldPosition(p1);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetPoint1WorldPosition(double p1[3])
{
  switch (this->WidgetState)
    {
    case vtkBoneWidget::Start:
      {
      std::cerr<<"Cannot Set Point1 Position in Start Mode. "
               <<"Use Rest Mode for this."
               <<std::endl<<"-> Doing Nothing"<<std::endl;
      break;
      }
    case vtkBoneWidget::Define:
      {
      this->GetvtkBoneRepresentation()->SetPoint1WorldPosition(p1);
      break;
      }
    case vtkBoneWidget::Rest:
      {
      this->GetvtkBoneRepresentation()->SetPoint1WorldPosition(p1);
      this->RebuildOrientation();
      this->RebuildLocalRestPoints();
      this->RebuildDebugAxes();
      this->RebuildParentageLink();

      this->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      break;
      }
    case vtkBoneWidget::Pose:
      {
      std::cerr<<"Cannot Set Point1 Position in Pose Mode."
               <<"Point 1 is assumed never to move. Use Rest Mode for this."
               <<std::endl<<"-> Doing Nothing"<<std::endl;
      break;
      }
    default:
      {
      std::cerr<<"Widget is in an unknown mode. Cannot set Point1"<<std::endl;
      break;
      }
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetPoint2WorldPosition(double p2[3])
{
  CopyVector3(
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(),
    p2);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetPoint2WorldPosition()
{
  return this->GetvtkBoneRepresentation()->GetPoint2WorldPosition();
}


//----------------------------------------------------------------------
void vtkBoneWidget::SetPoint2WorldPosition(double x, double y, double z)
{
   double p2[3];
   p2[0] = x;
   p2[1] = y;
   p2[2] = z;
   this->SetPoint2WorldPosition(p2);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetPoint2WorldPosition(double p2[3])
{
  switch (this->WidgetState)
    {
    case vtkBoneWidget::Start:
      {
      std::cerr<<"Cannot Set Point2 Position in Start Mode. "
               <<"Use Rest Mode for this."
               <<std::endl<<"-> Doing Nothing"<<std::endl;
      break;
      }
    case vtkBoneWidget::Define:
      {
      std::cerr<<"Cannot Set Point2 Position in Define Mode. "
               <<"Use Rest Mode for this."
               <<std::endl<<"-> Doing Nothing"<<std::endl;
      break;
      }
    case vtkBoneWidget::Rest:
      {
      this->GetvtkBoneRepresentation()->SetPoint2WorldPosition(p2);
      this->RebuildOrientation();
      this->RebuildLocalRestPoints();
      this->RebuildDebugAxes();
      this->RebuildParentageLink();


      this->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      break;
      }
    case vtkBoneWidget::Pose:
      {
      double p1[3], p2[3], lineVect[3];
      double distance = this->GetvtkBoneRepresentation()->GetDistance();

      this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(p1);
      this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(p2);

      vtkMath::Subtract(p2, p1, lineVect);
      vtkMath::Normalize(lineVect);
      vtkMath::MultiplyScalar(lineVect, distance);
      vtkMath::Add(p1, lineVect, p2);

      this->GetvtkBoneRepresentation()->SetPoint2WorldPosition(p2);

      this->RebuildPoseTransform();
      //this->RebuildLocalPoints();
      this->RebuildDebugAxes();
      this->RebuildParentageLink();

      this->InvokeEvent(vtkBoneWidget::PoseChangedEvent, NULL);
      break;
      }
    default:
      {
      std::cerr<<"Widget is in an unknown mode. Cannot set Point2"<<std::endl;
      break;
      }
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetBoneParent(vtkBoneWidget* parent)
{
  if (this->WidgetState == vtkBoneWidget::Pose)
    {
    std::cerr<<"Cannot define parent bone in pose mode."
      << std::endl << " ->Doing nothing"<<std::endl;
    return;
    }

  if (this->BoneParent)
    {
    this->BoneParent->RemoveObserver(vtkBoneWidget::RestChangedEvent);
    this->BoneParent->RemoveObserver(vtkBoneWidget::PoseChangedEvent);
    this->BoneParent->RemoveObserver(vtkBoneWidget::PoseInteractionStoppedEvent);
    }

  this->BoneParent = parent;

  if (parent)
    {
    parent->AddObserver(vtkBoneWidget::RestChangedEvent,
                        this->BoneWidgetChildrenCallback,
                        this->Priority);
    parent->AddObserver(vtkBoneWidget::PoseChangedEvent,
                        this->BoneWidgetChildrenCallback,
                        this->Priority);
    parent->AddObserver(vtkBoneWidget::PoseInteractionStoppedEvent,
                        this->BoneWidgetChildrenCallback,
                        this->Priority);

    if (this->P1LinkedToParent)
      {
      this->LinkPoint1ToParent();
      }
    this->RebuildParentageLink();

    this->RebuildLocalRestPoints();
    }
  else
    {
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(this->LocalRestP1);
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(this->LocalRestP2);
    }
}

//----------------------------------------------------------------------
vtkBoneWidget* vtkBoneWidget::GetBoneParent()
{
  return this->BoneParent;
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetOrientation(double orientation[4])
{
  CopyQuaternion(this->Orientation, orientation);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetPoseTransform()
{
  return this->PoseTransform;
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetPoseTransform(double poseTransform[4])
{
  CopyQuaternion(this->PoseTransform, poseTransform);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetOrientation()
{
  return this->Orientation;
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildOrientation()
{
  //Code greatly inspired by: http://www.fastgraph.com/makegames/3drotation/

  double p1[3], p2[3];
  this->GetvtkBoneRepresentation()->GetPoint1WorldPosition( p1 );
  this->GetvtkBoneRepresentation()->GetPoint2WorldPosition( p2 );

  double viewOut[3];      // the View or "new Z" vector
  double viewUp[3];       // the Up or "new Y" vector
  double viewRight[3];    // the Right or "new X" vector

  double upMagnitude;     // for normalizing the Up vector
  double upProjection;    // magnitude of projection of View Vector on World UP

  // first, calculate and normalize the view vector
  vtkMath::Subtract(p2, p1, viewOut);

  // normalize. This is the unit vector in the "new Z" direction
   // invalid points (not far enough apart)
  if (vtkMath::Normalize(viewOut) < 0.000001)
    {
    std::cerr<<"P2 and P1 are not enough apart,"
                " could not rebuild orientation"<<std::endl;
    InitializeQuaternion(this->Orientation);
    return;
    }

  // Now the hard part: The ViewUp or "new Y" vector

  // dot product of ViewOut vector and World Up vector gives projection of
  // of ViewOut on WorldUp
  upProjection = vtkMath::Dot(viewOut, Y);

  // first try at making a View Up vector: use World Up
  viewUp[0] = Y[0] - upProjection*viewOut[0];
  viewUp[1] = Y[1] - upProjection*viewOut[1];
  viewUp[2] = Y[2] - upProjection*viewOut[2];

  // Check for validity:
  upMagnitude = vtkMath::Norm(viewUp);

  if (upMagnitude < 0.0000001)
    {
    //Second try at making a View Up vector: Use Y axis default  (0,1,0)
    viewUp[0] = -viewOut[1]*viewOut[0];
    viewUp[1] = 1-viewOut[1]*viewOut[1];
    viewUp[2] = -viewOut[1]*viewOut[2];

    // Check for validity:
    upMagnitude = vtkMath::Norm(viewUp);

    if (upMagnitude < 0.0000001)
      {
      //Final try at making a View Up vector: Use Z axis default  (0,0,1)
      viewUp[0] = -viewOut[2]*viewOut[0];
      viewUp[1] = -viewOut[2]*viewOut[1];
      viewUp[2] = 1-viewOut[2]*viewOut[2];

      // Check for validity:
      upMagnitude = vtkMath::Norm(viewUp);

      if (upMagnitude < 0.0000001)
        {
        std::cerr<<"Could not fin a vector perpendiculare to the bone,"
                    " check the bone value(s)."
                    " This should not be happening."<<std::endl;
        return;
        }
      }
    }

  // normalize the Up Vector
  upMagnitude = vtkMath::Normalize(viewUp);

  // Calculate the Right Vector. Use cross product of Out and Up.
  vtkMath::Cross(viewUp, viewOut,  viewRight);
  vtkMath::Normalize(viewRight); //Let's be paranoid about the normalization

  //Get the orientation matrix
  AxisAngleToQuaternion(viewRight, acos(upProjection) , this->Orientation);
  NormalizeQuaternion(this->Orientation);

  //Get the roll matrix
  double rollQuad[4];
  AxisAngleToQuaternion(viewOut, this->Roll , rollQuad);
  NormalizeQuaternion(rollQuad);

  //Get final matrix
  MultiplyQuaternion(rollQuad, this->Orientation, this->Orientation);
  NormalizeQuaternion(this->Orientation);
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildLocalRestPoints()
{
  if (this->BoneParent)
    {
    double axis[3], angle, *p1, *p2;

    angle = QuaternionToAxisAngle(this->BoneParent->GetOrientation(), axis);
    vtkMath::Normalize(axis);

    vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
    T->Translate( this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition() );
    T->RotateWXYZ( vtkMath::DegreesFromRadians(angle), axis );
    T->Inverse();

    p1 = T->TransformDoublePoint(this->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    CopyVector3(p1, this->LocalRestP1);

    p2 = T->TransformDoublePoint(this->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    CopyVector3(p2, this->LocalRestP2);
    }
  else
    {
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition( this->LocalRestP1 );
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition( this->LocalRestP2 );
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildLocalPosePoints()
{
  if (this->BoneParent)
    {
    //Get final rotation /axis transform
    double resultTransform[4], axis[3];
    MultiplyQuaternion(this->BoneParent->GetPoseTransform(),
                       this->BoneParent->GetOrientation(),
                       resultTransform);
    NormalizeQuaternion(resultTransform);
    double angle = QuaternionToAxisAngle(resultTransform, axis);

    //Make transform
    vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
    T->Translate( this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition() );
    T->RotateWXYZ( vtkMath::DegreesFromRadians(angle), axis );
    T->Inverse();

    //Transform points
    double* p1 = T->TransformDoublePoint(this->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    CopyVector3(p1, this->LocalPoseP1);

    double* p2 = T->TransformDoublePoint(this->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    CopyVector3(p2, this->LocalPoseP2);
    }
  else
    {
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition( this->LocalPoseP1 );
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition( this->LocalPoseP2 );
    }

}

//----------------------------------------------------------------------
void vtkBoneWidget::SetEnabled(int enabling)
{
  // The handle widgets are not actually enabled until they are placed.
  // The handle widgets take their representation from the vtkBoneRepresentation.
  if ( enabling )
    {
    if ( this->WidgetState == vtkBoneWidget::Start )
      {
      if (this->WidgetRep)
        {
        this->WidgetRep->SetVisibility(0);
        }
      if (this->Point1Widget)
        {
        this->Point1Widget->GetRepresentation()->SetVisibility(0);
        }
      if (this->Point2Widget)
        {
        this->Point2Widget->GetRepresentation()->SetVisibility(0);
        }
      }
    else
      {
      if (this->WidgetRep)
        {
        this->WidgetRep->SetVisibility(1);
        }
      if (this->Point1Widget)
        {
        this->Point1Widget->GetRepresentation()->SetVisibility(1);
        }
      if (this->Point2Widget)
        {
        this->Point2Widget->GetRepresentation()->SetVisibility(1);
        }
      // The interactor must be set prior to enabling the widget.
      if (this->Interactor)
        {
        this->Point1Widget->SetInteractor(this->Interactor);
        this->Point2Widget->SetInteractor(this->Interactor);

        this->DebugX->SetInteractor(this->Interactor);
        this->DebugY->SetInteractor(this->Interactor);
        this->DebugZ->SetInteractor(this->Interactor);
        this->ParentageLink->SetInteractor(this->Interactor);
        }

      this->Point1Widget->SetEnabled(1);
      this->Point2Widget->SetEnabled(1);
      }

    if (this->Point1Widget)
      {
      this->Point1Widget->SetRepresentation(
        vtkBoneRepresentation::SafeDownCast
        (this->WidgetRep)->GetPoint1Representation());
      this->Point1Widget->SetInteractor(this->Interactor);
      this->Point1Widget->GetRepresentation()->SetRenderer(
        this->CurrentRenderer);
      }
    if (this->Point2Widget)
      {
      this->Point2Widget->SetRepresentation(
        vtkBoneRepresentation::SafeDownCast
        (this->WidgetRep)->GetPoint2Representation());
      this->Point2Widget->SetInteractor(this->Interactor);
      this->Point2Widget->GetRepresentation()->SetRenderer(
        this->CurrentRenderer);
      }
    }
  else //disabling widget
    {
    if (this->Point1Widget)
      {
      this->Point1Widget->SetEnabled(0);
      }
    if (this->Point2Widget)
      {
      this->Point2Widget->SetEnabled(0);
      }
    }

  this->Superclass::SetEnabled(enabling);
  this->RebuildDebugAxes();
}

//----------------------------------------------------------------------
int vtkBoneWidget::IsMeasureValid()
{
  if ( this->WidgetState == vtkBoneWidget::Rest ||
    this->WidgetState == vtkBoneWidget::Define)
    {
    return 1;
    }
  else
    {
    return 0;
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetRepresentation(vtkBoneRepresentation* r)
{
  if (this->WidgetState == vtkBoneWidget::Pose
    || this->WidgetState == vtkBoneWidget::Rest)
    {
    r->SetPoint1WorldPosition(
      this->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    r->SetPoint2WorldPosition(
      this->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    }
  else if (this->WidgetState == vtkBoneWidget::Define)
    {
    r->SetPoint1WorldPosition(
      this->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    }

  this->Superclass::SetWidgetRepresentation(
    reinterpret_cast<vtkWidgetRepresentation*>(r));
}

//-------------------------------------------------------------------------
void vtkBoneWidget::AddPointAction(vtkAbstractWidget *w)
{
  vtkBoneWidget *self = vtkBoneWidget::SafeDownCast(w);

  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);

  // If we are placing the first point it's easy
  if ( self->WidgetState == vtkBoneWidget::Start )
    {
    self->GrabFocus(self->EventCallbackCommand);
    self->WidgetState = vtkBoneWidget::Define;
    self->InvokeEvent(vtkCommand::StartInteractionEvent,NULL);

    if (self->P1LinkedToParent && self->BoneParent)
      {
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetPoint1WorldPosition(
        self->BoneParent->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
      }
    else
      {
      //Place Point yourself
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetPoint1DisplayPosition(e);
      self->Point1Widget->SetEnabled(1);
      }
    }

  // If defining we are placing the second or third point
  else if ( self->WidgetState == vtkBoneWidget::Define )
    {
    //Place Point
    self->WidgetState = vtkBoneWidget::Rest;

    vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetPoint2DisplayPosition(e);
    self->Point2Widget->SetEnabled(1);
    self->Point2Widget->GetRepresentation()->SetVisibility(1);
    self->WidgetRep->SetVisibility(1);

    self->RebuildOrientation();
    self->RebuildLocalRestPoints();
    self->RebuildDebugAxes();
    self->RebuildParentageLink();
    }

  else if ( self->WidgetState == vtkBoneWidget::Rest 
            || self->WidgetState == vtkBoneWidget::Pose )
    {
    self->BoneSelected = 0;
    self->Point1Selected = 0;
    self->Point2Selected = 0;

    int modifier = self->Interactor->GetShiftKey() | self->Interactor->GetControlKey();
    int state = self->WidgetRep->ComputeInteractionState(X,Y,modifier);
    if ( state == vtkBoneRepresentation::Outside )
      {
      return;
      }

    self->GrabFocus(self->EventCallbackCommand);
    if ( state == vtkBoneRepresentation::OnP1 )
      {
      self->GetvtkBoneRepresentation()->HighlightPoint(0, 1);
      self->Point1Selected = 1;
      self->InvokeEvent(vtkCommand::LeftButtonPressEvent,NULL);
      }
    else if (state == vtkBoneRepresentation::OnP2 )
      {
      self->GetvtkBoneRepresentation()->HighlightPoint(1, 1);
      self->Point2Selected = 1;
      self->InvokeEvent(vtkCommand::LeftButtonPressEvent,NULL);
      }
    else if ( state == vtkBoneRepresentation::OnLine)
      {
      if (self->WidgetState == vtkBoneWidget::Rest
          || !self->BoneParent)
        {
        self->GetvtkBoneRepresentation()->HighlightLine(1);
        self->BoneSelected = 1;
        self->WidgetRep->StartWidgetInteraction(e);
        self->InvokeEvent(vtkCommand::LeftButtonPressEvent,NULL);
        }
      }
    }

  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

//-------------------------------------------------------------------------
void vtkBoneWidget::MoveAction(vtkAbstractWidget *w)
{
  vtkBoneWidget *self = vtkBoneWidget::SafeDownCast(w);

  // Do nothing if outside
  if ( self->WidgetState == vtkBoneWidget::Start )
    {
    return;
    }

  // Delegate the event consistent with the state
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);

  if ( self->WidgetState == vtkBoneWidget::Define )
    {
    self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
    self->EventCallbackCommand->SetAbortFlag(1);
    }

  else if (self->WidgetState == vtkBoneWidget::Rest)
    {
    if ( self->Point1Selected )
      {
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetPoint1DisplayPosition(e);
      self->RebuildOrientation();
      self->RebuildLocalRestPoints();
      self->RebuildDebugAxes();
      self->RebuildParentageLink();

      //self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }

    else if ( self->Point2Selected )
      {
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetPoint2DisplayPosition(e);
      self->RebuildOrientation();
      self->RebuildLocalRestPoints();
      self->RebuildDebugAxes();
      self->RebuildParentageLink();

      self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }

    else if ( self->BoneSelected )
      {
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)
        ->GetLineHandleRepresentation()->SetDisplayPosition(e);
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->WidgetInteraction(e);
      self->RebuildOrientation();
      self->RebuildLocalRestPoints();
      self->RebuildDebugAxes();
      self->RebuildParentageLink();

      if (self->P1LinkedToParent && self->BoneParent)
        {
        self->BoneParent->LinkParentPoint2To(self);
        }

      self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }
    }

  else if (self->WidgetState == vtkBoneWidget::Pose)
    {
    //Cannot move P1 in pose mode

    if ( self->Point2Selected )
      {
      //
      //Make rotation in camera view plane center on p1
      //

      // Get display positions
      double e1[2], e2[2];
      self->GetvtkBoneRepresentation()->GetPoint1DisplayPosition(e1);
      self->GetvtkBoneRepresentation()->GetPoint2DisplayPosition(e2);

      // Get the current line -> the line between p1 and the event
      //in display coordinates
      double currentLine[2], oldLine[2];
      currentLine[0] = e[0] - e1[0]; currentLine[1] = e[1] - e1[1];
      vtkMath::Normalize2D(currentLine);

      // Get the old line -> the line between p1 and the LAST event
      //in display coordinates
      int lastX = self->Interactor->GetLastEventPosition()[0];
      int lastY = self->Interactor->GetLastEventPosition()[1];
      double lastE[2];
      lastE[0] = static_cast<double>(lastX);
      lastE[1] = static_cast<double>(lastY);
      oldLine[0] = lastE[0] - e1[0]; oldLine[1] = lastE[1] - e1[1];
      vtkMath::Normalize2D(oldLine);

      // Get the angle between those two lines
      double angle = vtkMath::DegreesFromRadians(
                       acos(vtkMath::Dot2D(currentLine, oldLine)));

      // Get the world coordinate of the line before anything moves
      double p1[3], p2[3];
      self->GetvtkBoneRepresentation()->GetPoint1WorldPosition(p1);
      self->GetvtkBoneRepresentation()->GetPoint2WorldPosition(p2);

      //Get the camera vector
      double cameraVec[3];
      if (!self->GetCurrentRenderer()
          || !self->GetCurrentRenderer()->GetActiveCamera())
        {
        std::cerr<<"There should be a renderer and a camera."
                 << " Make sure to set these !"<<std::endl
                 << "->Cannot move P2 in pose mode"<<std::endl;
        return;
        }
      self->GetCurrentRenderer()->GetActiveCamera()->GetDirectionOfProjection(cameraVec);

      //Need to figure if the rotation is clockwise or counterclowise
      double spaceCurrentLine[3], spaceOldLine[3];
      spaceCurrentLine[0] = currentLine[0];
      spaceCurrentLine[1] = currentLine[1];
      spaceCurrentLine[2] = 0.0;

      spaceOldLine[0] = oldLine[0];
      spaceOldLine[1] = oldLine[1];
      spaceOldLine[2] = 0.0;

      double handenessVec[3];
      vtkMath::Cross(spaceOldLine, spaceCurrentLine, handenessVec);

      //handeness is oppostie beacuse camera is toward the focal point
      double handeness = vtkMath::Dot(handenessVec, Z) > 0 ? -1.0: 1.0;
      angle *= handeness;

      //Finally rotate P2
      vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
      T->Translate(p1); //last transform: Translate back to P1 origin
      T->RotateWXYZ(angle, cameraVec); //middle transform: rotate
      //first transform: Translate to world origin
      double minusP1[3];
      CopyVector3(p1, minusP1);
      vtkMath::MultiplyScalar(minusP1, -1.0);
      T->Translate(minusP1);

      self->GetvtkBoneRepresentation()->SetPoint2WorldPosition(
        T->TransformDoublePoint(p2));

      self->RebuildPoseTransform();
      self->RebuildLocalPosePoints();
      self->RebuildDebugAxes();
      self->RebuildParentageLink();

      self->InvokeEvent(vtkBoneWidget::PoseChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }
    else if ( self->BoneSelected )
      {
      if (!self->BoneParent) //shouldn't be necessary since the
                             //sorting is done in AddAction but just in case
        {
        // moving outer portion of line -- rotating
        vtkBoneRepresentation::SafeDownCast(self->WidgetRep)
          ->GetLineHandleRepresentation()->SetDisplayPosition(e);
        vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->WidgetInteraction(e);

        self->RebuildPoseTransform();
        self->RebuildLocalPosePoints();
        self->RebuildDebugAxes();
        self->RebuildParentageLink();

        self->InvokeEvent(vtkBoneWidget::PoseChangedEvent, NULL);
        self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
        }
      }
    }

  self->WidgetRep->BuildRepresentation();
  self->Render();
}

//-------------------------------------------------------------------------
void vtkBoneWidget::EndSelectAction(vtkAbstractWidget *w)
{
  vtkBoneWidget *self = vtkBoneWidget::SafeDownCast(w);

  // Do nothing if outside
  if ( self->WidgetState == vtkBoneWidget::Start ||
       self->WidgetState == vtkBoneWidget::Define )
    {
    return;
    }

  if ( self->WidgetState == vtkBoneWidget::Pose )
    {
    self->BoneParentInteractionStopped();
    }

  self->BoneSelected = 0;
  self->Point1Selected = 0;
  self->Point2Selected = 0;
  self->WidgetRep->Highlight(0);
  self->ReleaseFocus();
  self->WidgetRep->BuildRepresentation();
  int state = self->WidgetRep->GetInteractionState();
  if ( state == vtkBoneRepresentation::OnP1 ||
       state == vtkBoneRepresentation::OnP2 )
    {
    self->InvokeEvent(vtkCommand::LeftButtonReleaseEvent,NULL);
    }
  else
    {
    self->EndBoneInteraction();
    }
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

//-------------------------------------------------------------------------
void vtkBoneWidget::BoneParentInteractionStopped()
{
  //If the movement is finished, store the pose transform
  CopyQuaternion(this->PoseTransform, this->OldPoseTransform);

  //And update the pose point
  this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(this->TemporaryPoseP1);
  this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(this->TemporaryPoseP2);

  this->InvokeEvent(vtkBoneWidget::PoseInteractionStoppedEvent, NULL);
}

//-------------------------------------------------------------------------
void vtkBoneWidget::BoneParentPoseChanged()
{
   if (this->BoneParent)
    {
    double  axis[3], resultTransform[4];

    //1- multiply quaternion
    MultiplyQuaternion(this->BoneParent->GetPoseTransform(),
                       this->BoneParent->GetOrientation(),
                       resultTransform);
    NormalizeQuaternion(resultTransform);

    //2- Get axis and angle
    double angle = QuaternionToAxisAngle(resultTransform, axis);
    vtkMath::Normalize(axis);

    //3- transform point
    vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
    T->Translate(this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    T->RotateWXYZ(vtkMath::DegreesFromRadians(angle), axis);

    double* newP1 = T->TransformDoublePoint(this->LocalPoseP1);
    this->GetvtkBoneRepresentation()->SetPoint1WorldPosition(newP1);

    double* newP2 = T->TransformDoublePoint(this->LocalPoseP2);
    this->GetvtkBoneRepresentation()->SetPoint2WorldPosition(newP2);

    this->RebuildPoseTransform();
    this->RebuildDebugAxes();
    this->RebuildParentageLink();
    this->InvokeEvent(vtkBoneWidget::PoseChangedEvent, NULL);
    }
}

//-------------------------------------------------------------------------
void vtkBoneWidget::RebuildPoseTransform()
{
  if (this->WidgetState != vtkBoneWidget::Pose)
    {
    InitializeQuaternion(this->PoseTransform);
    }
  else
    {
    // Cumulative technique is simple but causes drift :(
    // That is why we need to recompute each time.
    // The old pose transform represents the sum of all the other
    // previous transformations.

    double p1[3], p2[3], previousLineVect[3], newLineVect[3], rotationAxis[3], poseAngle;
    // 1- Get P1, P2, Old P2
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition( p1 );
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition( p2 );

    // 2- Get the previous line directionnal vector
    vtkMath::Subtract(this->TemporaryPoseP2, this->TemporaryPoseP1, previousLineVect);
    vtkMath::Normalize(previousLineVect);

    // 3- Get the new line vector
    vtkMath::Subtract(p2, p1, newLineVect);
      vtkMath::Normalize(newLineVect);

    if (this->GetCurrentRenderer() && this->GetCurrentRenderer()->GetActiveCamera())
      {
      // 4- Compute Rotation Axis
      this->GetCurrentRenderer()->GetActiveCamera()->GetDirectionOfProjection(rotationAxis);
      vtkMath::Normalize(rotationAxis);//Let's be paranoid about normalization

      // 4- Compute Angle
      double rotationPlaneAxis1[3], rotationPlaneAxis2[3];
      vtkMath::Perpendiculars(rotationAxis, rotationPlaneAxis1, rotationPlaneAxis2, 0.0);
      vtkMath::Normalize(rotationPlaneAxis1);//Let's be paranoid about normalization
      vtkMath::Normalize(rotationPlaneAxis2);

      //The angle is the difference between the old angle and the new angle.
      //Doing this difference enables us to not care about the possible roll
      //of the camera
      double newVectAngle = atan2(vtkMath::Dot(newLineVect, rotationPlaneAxis2),
                                  vtkMath::Dot(newLineVect, rotationPlaneAxis1));
      double previousVectAngle = atan2(vtkMath::Dot(previousLineVect, rotationPlaneAxis2),
                                       vtkMath::Dot(previousLineVect, rotationPlaneAxis1));
      poseAngle = newVectAngle - previousVectAngle;
      }
    else //this else is here for now but maybe we should just output an error message ?
         //beacuse I do not think this code would work properly. 
      {
      // 4- Compute Rotation Axis
      vtkMath::Cross(previousLineVect, newLineVect, rotationAxis);
      vtkMath::Normalize(rotationAxis);

      // 4- Compute Angle
      poseAngle = acos(vtkMath::Dot(newLineVect, previousLineVect));
      }

    // PoseTransform is the sum of the transform applied to the bone in
    // pose mode. The previous transform are stored in OldPoseTransform
    double quad[4];
    AxisAngleToQuaternion(rotationAxis, poseAngle, quad);
    NormalizeQuaternion(quad);
    MultiplyQuaternion(quad, this->OldPoseTransform, this->PoseTransform);
    NormalizeQuaternion(this->PoseTransform);
    }

}

//-------------------------------------------------------------------------
void vtkBoneWidget::BoneParentRestChanged()
{
  //In the previous behavior, we had the child P1 to follow the parent
  //P2 in distance

  //Now they either are stuck together or nothing
  if (this->P1LinkedToParent)
    {
    this->LinkPoint1ToParent();
    }
  this->RebuildParentageLink();
}

//----------------------------------------------------------------------
void vtkBoneWidget::StartBoneInteraction()
{
  this->Superclass::StartInteraction();
  this->InvokeEvent(vtkCommand::StartInteractionEvent,NULL);
}

//----------------------------------------------------------------------
void vtkBoneWidget::EndBoneInteraction()
{
  this->Superclass::EndInteraction();
  this->InvokeEvent(vtkCommand::EndInteractionEvent,NULL);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetProcessEvents(int pe)
{
  this->Superclass::SetProcessEvents(pe);

  this->Point1Widget->SetProcessEvents(pe);
  this->Point2Widget->SetProcessEvents(pe);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetState(int state)
{
  switch (state)
    {
    case vtkBoneWidget::Start:
      {
      this->SetWidgetStateToStart();
      break;
      }
    case vtkBoneWidget::Define:
      {
      std::cerr<<"Cannot set state to Define from outside this class"
        <<std::endl<<" -> Doing nothing."<<std::endl;
      break;
      }
    case vtkBoneWidget::Rest:
      {
      this->SetWidgetStateToRest();
      break;
      }
    case vtkBoneWidget::Pose:
      {
      this->SetWidgetStateToPose();
      break;
      }
    default:
      {
      std::cerr<<"Unknown state. The only possible values are:"
        <<"    0 <-> Start"<<std::endl
        <<"    2 <-> Rest"<<std::endl
        <<"    3 <-> Pose"<<std::endl
        <<" -> Doing nothing."<<std::endl;
      break;
      }
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetStateToStart()
{
  this->WidgetState = vtkBoneWidget::Start;
  this->BoneSelected = 0;
  this->Point1Selected = 0;
  this->Point2Selected = 0;
  
  this->WidgetRep->Highlight(0);

  this->RebuildDebugAxes();
  this->RebuildParentageLink();
  this->SetEnabled(this->GetEnabled()); // show/hide the handles properly
  this->ReleaseFocus();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetStateToPose()
{
  this->BoneSelected = 0;
  this->Point1Selected = 0;
  this->Point2Selected = 0;

  CopyVector3(this->LocalRestP1, this->LocalPoseP1);
  CopyVector3(this->LocalRestP2, this->LocalPoseP2);
  InitializeQuaternion(this->OldPoseTransform);

  this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(this->TemporaryPoseP1);
  this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(this->TemporaryPoseP2);

  if (this->WidgetState != vtkBoneWidget::Rest)
    {
    this->RebuildOrientation();
    }
  this->WidgetState = vtkBoneWidget::Pose;

  this->RebuildPoseTransform();
  this->RebuildDebugAxes();
  this->RebuildParentageLink();

  this->SetEnabled(this->GetEnabled()); // show/hide the handles properly
  this->ReleaseFocus();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetStateToRest()
{
  this->BoneSelected = 0;
  this->Point1Selected = 0;
  this->Point2Selected = 0;

  InitializeQuaternion(this->PoseTransform);
  InitializeQuaternion(this->OldPoseTransform);
  InitializeVector3(this->TemporaryPoseP1);
  InitializeVector3(this->TemporaryPoseP2);
  InitializeVector3(this->LocalPoseP1);
  InitializeVector3(this->LocalPoseP2);

  if (this->P1LinkedToParent)
    {
    this->LinkPoint1ToParent();
    }

  if (this->WidgetState != vtkBoneWidget::Pose)
    {
    this->RebuildOrientation();
    this->RebuildLocalRestPoints();
    }
  else // previous state was pose
    {
    //We need to reset the points to their original rest position
    if (this->BoneParent)
      {
      //Reset point to original rest position
      double axis[3];
      double angle = QuaternionToAxisAngle(
        this->BoneParent->GetOrientation(), axis);
      vtkMath::Normalize(axis);

      vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();

      T->Translate(
        this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
      T->RotateWXYZ(vtkMath::DegreesFromRadians(angle), axis);

      double* newP1 = T->TransformDoublePoint(this->LocalRestP1);
      this->GetvtkBoneRepresentation()->SetPoint1WorldPosition(newP1);

      double* newP2 = T->TransformDoublePoint(this->LocalRestP2);
      this->GetvtkBoneRepresentation()->SetPoint2WorldPosition(newP2);
      }
    else
      {
      this->GetvtkBoneRepresentation()->SetPoint1WorldPosition(this->LocalRestP1);
      this->GetvtkBoneRepresentation()->SetPoint2WorldPosition(this->LocalRestP2);
      }
    }

  this->WidgetState = vtkBoneWidget::Rest;

  this->RebuildDebugAxes();
  this->RebuildParentageLink();
  this->SetEnabled(this->GetEnabled()); // show/hide the handles properly
  this->ReleaseFocus();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetDebugAxes(int debugAxes)
{
  this->DebugAxes = debugAxes;
  this->RebuildDebugAxes();
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildParentageLink()
{
  vtkLineRepresentation* rep=
      vtkLineRepresentation::SafeDownCast(
        this->ParentageLink->GetRepresentation());

  if (this->ShowParentage && this->BoneParent && ! this->P1LinkedToParent
      && (this->WidgetState == vtkBoneWidget::Rest
          || this->WidgetState == vtkBoneWidget::Pose))
    {
    rep->SetVisibility(1);

    rep->SetPoint1WorldPosition(
      this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    rep->SetPoint2WorldPosition(
      this->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    }
  else
    {
    rep->SetVisibility(0);
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildDebugAxes()
{
  if (this->DebugAxes == vtkBoneWidget::Nothing
      || this->WidgetState == vtkBoneWidget::Start
      || this->WidgetState == vtkBoneWidget::Define)
    {
    if (vtkLineRepresentation::SafeDownCast(
          this->DebugX->GetRepresentation())->GetVisibility() == 1)
      {
      vtkLineRepresentation::SafeDownCast(
        this->DebugX->GetRepresentation())->SetVisibility(0);
      vtkLineRepresentation::SafeDownCast(
        this->DebugY->GetRepresentation())->SetVisibility(0);
      vtkLineRepresentation::SafeDownCast(
        this->DebugZ->GetRepresentation())->SetVisibility(0);
      }
    }
  else if (this->DebugAxes == vtkBoneWidget::ShowOrientation
           || this->DebugAxes == vtkBoneWidget::ShowPoseTransform
           || this->DebugAxes == vtkBoneWidget::ShowPoseTransformAndOrientation)
    {
    double o[3], axis[3], angle;
    double distance =
      this->GetvtkBoneRepresentation()->GetDistance() * this->DebugAxesSize;
    this->GetvtkBoneRepresentation()->GetPoint2WorldPosition(o);
      
    if (this->DebugAxes == vtkBoneWidget::ShowOrientation)
      {
      angle = vtkBoneWidget::QuaternionToAxisAngle(this->Orientation, axis);
      }
    else if (this->DebugAxes == vtkBoneWidget::ShowPoseTransform)
      {
      angle = vtkBoneWidget::QuaternionToAxisAngle(this->PoseTransform, axis);
      }
    else if (this->DebugAxes == vtkBoneWidget::ShowPoseTransformAndOrientation)
      {
      double resultTransform[4];
      MultiplyQuaternion(this->GetPoseTransform(),
                         this->GetOrientation(),
                          resultTransform);
      NormalizeQuaternion(resultTransform);

      angle = vtkBoneWidget::QuaternionToAxisAngle(resultTransform, axis);
      }

    vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
    T->Translate( o );
    T->RotateWXYZ( vtkMath::DegreesFromRadians(angle), axis );

    vtkLineRepresentation::SafeDownCast(
      this->DebugX->GetRepresentation())->SetPoint1WorldPosition(o);
    vtkLineRepresentation::SafeDownCast(
      this->DebugX->GetRepresentation())->SetPoint2WorldPosition(
        T->TransformDoublePoint(distance, 0.0, 0.0));

    vtkLineRepresentation::SafeDownCast(
      this->DebugY->GetRepresentation())->SetPoint1WorldPosition(o);
    vtkLineRepresentation::SafeDownCast(
      this->DebugY->GetRepresentation())->SetPoint2WorldPosition(
        T->TransformDoublePoint(0.0, distance, 0.0));

    vtkLineRepresentation::SafeDownCast(
      this->DebugZ->GetRepresentation())->SetPoint1WorldPosition(o);
    vtkLineRepresentation::SafeDownCast(
      this->DebugZ->GetRepresentation())->SetPoint2WorldPosition(
        T->TransformDoublePoint(0.0, 0.0, distance));

    if (vtkLineRepresentation::SafeDownCast(
          this->DebugX->GetRepresentation())->GetVisibility() == 0)
      {
      vtkLineRepresentation::SafeDownCast(
        this->DebugX->GetRepresentation())->SetVisibility(1);
      vtkLineRepresentation::SafeDownCast(
        this->DebugY->GetRepresentation())->SetVisibility(1);
      vtkLineRepresentation::SafeDownCast(
        this->DebugZ->GetRepresentation())->SetVisibility(1);
      }
    }
  else
    {
    std::cerr<<"Unknow value for DebugAxes. ->Doing nothing."<<std::endl;
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetP1LinkedToParent(int link)
{
  if (link)
    {
    if (!this->BoneParent)
      {
      std::cerr<<"Cannot link P1 to a non-existing parent."
                " Set the bone parent before. ->Doing nothing."<<std::endl;
      }

    //Disable P1
    this->Point1Selected = 0;
    this->Point1Widget->SetEnabled(0);

    this->LinkPoint1ToParent();
    }
  else
    {
    this->Point1Widget->SetEnabled(1);
    }

  this->P1LinkedToParent = link;
}

//----------------------------------------------------------------------
void vtkBoneWidget::LinkPoint1ToParent()
{
  //Move this Point1 to Follow the parent movement
  if (this->BoneParent)
    {
    this->SetPoint1WorldPosition(
      this->BoneParent->GetvtkBoneRepresentation()->GetPoint2WorldPosition());
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::LinkParentPoint2To(vtkBoneWidget* child)
{
  // Move this point to snap to given child
  if (child->GetP1LinkedToParent()) //never too sure
    //(I could even verify the child's parent is indeed this)
    {
    this->SetPoint2WorldPosition(
      child->GetvtkBoneRepresentation()->GetPoint1WorldPosition());
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetShowParentage(int parentage)
{
  this->ShowParentage = parentage;
  this->RebuildParentageLink();
}

//----------------------------------------------------------------------
void vtkBoneWidget::AxisAngleToQuaternion(double axis[3], double angle, double quad[4])
{
  quad[0] = cos( angle / 2.0 );
  double f = sin( angle / 2.0);

  double vec[3];
  vec[0] = axis[0];
  vec[1] = axis[1];
  vec[2] = axis[2];
  vtkMath::Normalize(vec);

  quad[1] =  vec[0] * f;
  quad[2] =  vec[1] * f;
  quad[3] =  vec[2] * f;
}

//----------------------------------------------------------------------
vtkTransform* vtkBoneWidget::GetWorldToBoneTransform()
{
  if (this->WidgetState == vtkBoneWidget::Start
      || this->WidgetState == vtkBoneWidget::Define)
    {
    return NULL;
    }
  else
    {
    double o[3], axis[3], angle;
    this->GetvtkBoneRepresentation()->GetPoint1WorldPosition(o);

    vtkTransform* T = vtkTransform::New();
    T->Translate( o );

    if (this->WidgetState == Rest)
      {
      angle = QuaternionToAxisAngle(this->Orientation, axis);
      }
    else //Pose mode
      {
      double resultTransform[4];
      MultiplyQuaternion(this->BoneParent->GetPoseTransform(),
                       this->BoneParent->GetOrientation(),
                       resultTransform);
      NormalizeQuaternion(resultTransform);

      angle = QuaternionToAxisAngle(resultTransform, axis);
      }
    T->RotateWXYZ(angle, axis);
    return T;
    }
}

//----------------------------------------------------------------------
double vtkBoneWidget::QuaternionToAxisAngle(double quad[4], double axis[3])
{
  double angle = acos(quad[0]) *2.0;
  double f = sin( angle * 0.5 );
  if (f > 1e-13)
    {
    axis[0] = quad[1] / f;
    axis[1] = quad[2] / f;
    axis[2] = quad[3] / f;
    }
  else
    {
    if (angle > 1e-13 || angle < 1e-13) //means rotation of pi
      {
      axis[0] = 1.0;
      axis[1] = 0.0;
      axis[2] = 0.0;
      }
    else
      {
      axis[0] = 0.0;
      axis[1] = 0.0;
      axis[2] = 0.0;
      }
    }

  return angle;
}

//----------------------------------------------------------------------
void vtkBoneWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "Bone Widget " << this << "\n";

  os << indent << "Widget State: "<< this->WidgetState<< "\n";

  os << indent << "Selected:"<< "\n";
  os << indent << "  Bone Selected: "<< this->BoneSelected<< "\n";
  os << indent << "  P1 Selected: "<< this->Point1Selected<< "\n";
  os << indent << "  P2 Selected: "<< this->Point2Selected<< "\n";

  if (this->BoneParent)
    {
    os << indent << "Bone Parent: "<< this->BoneParent << "\n";
    }

  os << indent << "Local Points:" << "\n";
  os << indent << "  Local Rest P1: "<< this->LocalRestP1[0]
                                << "  " << this->LocalRestP1[1]
                                << "  " << this->LocalRestP1[2]<< "\n";
  os << indent << "  Local Rest P2: "<< this->LocalRestP2[0]
                                << "  " << this->LocalRestP2[1]
                                << "  " << this->LocalRestP2[2]<< "\n";
  os << indent << "  Local Pose P1: "<< this->LocalPoseP1[0]
                                << "  " << this->LocalPoseP1[1]
                                << "  " << this->LocalPoseP1[2]<< "\n";
  os << indent << "  Local Pose P2: "<< this->LocalPoseP2[0]
                                << "  " << this->LocalPoseP2[1]
                                << "  " << this->LocalPoseP2[2]<< "\n";

  os << indent << "Temporary Points:" << "\n";
  os << indent << "  Temporary Pose P1: "<< this->TemporaryPoseP1[0]
                                << "  " << this->TemporaryPoseP1[1]
                                << "  " << this->TemporaryPoseP1[2]<< "\n";
  os << indent << "  Temporary Pose P2: "<< this->TemporaryPoseP2[0]
                                << "  " << this->TemporaryPoseP2[1]
                                << "  " << this->TemporaryPoseP2[2]<< "\n";

  os << indent << "Tranforms:" << "\n";
  os << indent << "  Orientation: "<< this->Orientation[0]
                                << "  " << this->Orientation[1]
                                << "  " << this->Orientation[2]
                                << "  " << this->Orientation[3]<< "\n";
  os << indent << "  PoseTransform: "<< this->PoseTransform[0]
                                << "  " << this->PoseTransform[1]
                                << "  " << this->PoseTransform[2]
                                << "  " << this->PoseTransform[3]<< "\n";
  os << indent << "  OldPoseTransform: "<< this->OldPoseTransform[0]
                                << "  " << this->OldPoseTransform[1]
                                << "  " << this->OldPoseTransform[2]
                                << "  " << this->OldPoseTransform[3]<< "\n";

  os << indent << "Roll: "<< this->Roll << "\n";

  os << indent << "Parent link: "<< "\n";
  os << indent << "  P1LinkToParent: "<< this->P1LinkedToParent << "\n";
  os << indent << "  ShowParentage: "<< this->ShowParentage << "\n";

  os << indent << "Debug:" << "\n";
  os << indent << "  Debug Axes: "<< this->DebugAxes << "\n";
  os << indent << "  Debug Axes Size: "<< this->DebugAxesSize << "\n";
}
