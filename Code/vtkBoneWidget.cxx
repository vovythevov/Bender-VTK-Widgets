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
#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
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
  vtkBoneWidgetCallback()
    { this->BoneWidget = 0; }
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

  //Widget interaction init
  this->WidgetState = vtkBoneWidget::Start;
  this->BoneSelected = 0;
  this->HeadSelected = 0;
  this->TailSelected = 0;

  // The widgets for moving the end points. They observe this widget (i.e.,
  // this widget is the parent to the handles).
  this->HeadWidget = vtkHandleWidget::New();
  this->HeadWidget->SetPriority(this->Priority-0.01);
  this->HeadWidget->SetParent(this);
  this->HeadWidget->ManagesCursorOff();

  this->TailWidget = vtkHandleWidget::New();
  this->TailWidget->SetPriority(this->Priority-0.01);
  this->TailWidget->SetParent(this);
  this->TailWidget->ManagesCursorOff();

  // Set up the callbacks on the two handles
  this->BoneWidgetCallback1 = vtkBoneWidgetCallback::New();
  this->BoneWidgetCallback1->BoneWidget = this;
  this->HeadWidget->AddObserver(vtkCommand::StartInteractionEvent, this->BoneWidgetCallback1,
                                  this->Priority);
  this->HeadWidget->AddObserver(vtkCommand::EndInteractionEvent, this->BoneWidgetCallback1,
                                  this->Priority);

  this->BoneWidgetCallback2 = vtkBoneWidgetCallback::New();
  this->BoneWidgetCallback2->BoneWidget = this;
  this->TailWidget->AddObserver(vtkCommand::StartInteractionEvent, this->BoneWidgetCallback2,
                                  this->Priority);
  this->TailWidget->AddObserver(vtkCommand::EndInteractionEvent, this->BoneWidgetCallback2,
                                  this->Priority);

//Setup the callback for the parent/child processing
  this->BoneParent = NULL;

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

  //Init bone variables
  InitializeVector3(this->LocalRestHead);
  InitializeVector3(this->LocalRestTail);
  InitializeVector3(this->LocalPoseHead);
  InitializeVector3(this->LocalPoseTail);
  InitializeVector3(this->InteractionWorldHead);
  InitializeVector3(this->InteractionWorldTail);
  this->Roll = 0.0;
  InitializeQuaternion(this->RestTransform);
  InitializeQuaternion(this->PoseTransform);

  //parentage link init
  this->HeadLinkedToParent = 0;
  this->ShowParentage = 0;
  this->ParentageLink = vtkLineWidget2::New();

  //Debug axes init
  this->AxesVisibility = vtkBoneWidget::Nothing;
  this->AxesActor = vtkAxesActor::New();
  this->AxesActor->SetAxisLabels(0);
  this->AxesSize = 0.2;

  this->UpdateAxesVisibility();
  this->RebuildParentageLink();
}

//----------------------------------------------------------------------
vtkBoneWidget::~vtkBoneWidget()
{
  if(this->CurrentRenderer)
    {
    this->CurrentRenderer->RemoveActor(this->AxesActor);
    }
  this->AxesActor->Delete();

  this->ParentageLink->Delete();

  this->HeadWidget->RemoveObserver(this->BoneWidgetCallback1);
  this->HeadWidget->Delete();
  this->BoneWidgetCallback1->Delete();

  this->TailWidget->RemoveObserver(this->BoneWidgetCallback2);
  this->TailWidget->Delete();
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
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetHeadWorldPosition(double Head[3])
{
  CopyVector3(this->GetBoneRepresentation()->GetHeadWorldPosition(),Head);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetHeadWorldPosition()
{
  return this->GetBoneRepresentation()->GetHeadWorldPosition();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetHeadWorldPosition(double x, double y, double z)
{
   double head[3];
   head[0] = x;
   head[1] = y;
   head[2] = z;
   this->SetHeadWorldPosition(head);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetHeadWorldPosition(double head[3])
{
  double diff[3];
  vtkMath::Subtract(head,
                    this->GetBoneRepresentation()->GetHeadWorldPosition(),
                    diff);
  if (vtkMath::Norm(diff) < 1e-13)
    {
    return;
    }

  if (this->WidgetState == vtkBoneWidget::Pose)
    {
    vtkErrorMacro("Cannot set tail position in head mode."
                  " Use the interaction of the rotation methods instead");
    return;
    }

  this->GetBoneRepresentation()->SetHeadWorldPosition(head);

  if (this->WidgetState == vtkBoneWidget::Rest)
    {
    this->RebuildRestTransform();
    this->RebuildLocalRestPoints();

    this->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
    }

  this->RebuildAxes();
  this->RebuildParentageLink();
  this->Modified();
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetTailWorldPosition(double tail[3])
{
  CopyVector3(
    this->GetBoneRepresentation()->GetTailWorldPosition(),
    tail);
}

//----------------------------------------------------------------------
double* vtkBoneWidget::GetTailWorldPosition()
{
  return this->GetBoneRepresentation()->GetTailWorldPosition();
}


//----------------------------------------------------------------------
void vtkBoneWidget::SetTailWorldPosition(double x, double y, double z)
{
   double tail[3];
   tail[0] = x;
   tail[1] = y;
   tail[2] = z;
   this->SetTailWorldPosition(tail);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetTailWorldPosition(double tail[3])
{
  double diff[3];
  vtkMath::Subtract(tail,
                    this->GetBoneRepresentation()->GetTailWorldPosition(),
                    diff);
  if (vtkMath::Norm(diff) < 1e-13)
    {
    return;
    }

  if (this->WidgetState == vtkBoneWidget::Pose)
    {
    vtkErrorMacro("Cannot set tail position in pose mode."
                  " Use the interaction of the rotation methods instead");
    return;
    }

  this->GetBoneRepresentation()->SetTailWorldPosition(tail);

  if (this->WidgetState == vtkBoneWidget::Rest)
    {
    this->RebuildRestTransform();
    this->RebuildLocalRestPoints();

    this->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
    }

  this->RebuildAxes();
  this->RebuildParentageLink();

  this->Modified();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetBoneParent(vtkBoneWidget* parent)
{
  if (this->WidgetState == vtkBoneWidget::Pose)
    {
    vtkErrorMacro("Cannot define parent bone in pose mode."
                  "\n ->Doing nothing");
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

    if (this->HeadLinkedToParent)
      {
      this->LinkHeadToParent();
      }
    this->RebuildParentageLink();

    this->RebuildLocalRestPoints();
    }
  else
    {
    this->GetBoneRepresentation()->GetHeadWorldPosition(this->LocalRestHead);
    this->GetBoneRepresentation()->GetTailWorldPosition(this->LocalRestTail);
    }
}

//----------------------------------------------------------------------
vtkBoneWidget* vtkBoneWidget::GetBoneParent()
{
  return this->BoneParent;
}

//----------------------------------------------------------------------
void vtkBoneWidget::GetRestTransform(double restTransform[4])
{
  CopyQuaternion(this->RestTransform, restTransform);
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
double* vtkBoneWidget::GetRestTransform()
{
  return this->RestTransform;
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildRestTransform()
{
  //Code greatly inspired by: http://www.fastgraph.com/makegames/3drotation/

  double head[3], tail[3];
  this->GetBoneRepresentation()->GetHeadWorldPosition( head );
  this->GetBoneRepresentation()->GetTailWorldPosition( tail );

  double viewOut[3];      // the View or "new Z" vector
  double viewUp[3];       // the Up or "new Y" vector
  double viewRight[3];    // the Right or "new X" vector

  double upMagnitude;     // for normalizing the Up vector
  double upProjection;    // magnitude of projection of View Vector on World UP

  // first, calculate and normalize the view vector
  vtkMath::Subtract(tail, head, viewOut);

  // normalize. This is the unit vector in the "new Z" direction
  // invalid points (not far enough apart)
  if (vtkMath::Normalize(viewOut) < 0.000001)
    {
    vtkErrorMacro("Tail and Head are not enough apart,"
                  " could not rebuild rest Transform");
    InitializeQuaternion(this->RestTransform);
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
        vtkErrorMacro("Could not fin a vector perpendiculare to the bone,"
                      " check the bone values. This should not be happening.");
        return;
        }
      }
    }

  // normalize the Up Vector
  upMagnitude = vtkMath::Normalize(viewUp);

  // Calculate the Right Vector. Use cross product of Out and Up.
  vtkMath::Cross(viewUp, viewOut,  viewRight);
  vtkMath::Normalize(viewRight); //Let's be paranoid about the normalization

  //Get the rest transform matrix
  AxisAngleToQuaternion(viewRight, acos(upProjection) , this->RestTransform);
  NormalizeQuaternion(this->RestTransform);

  if (this->Roll != 0.0)
    {
    //Get the roll matrix
    double rollQuad[4];
    AxisAngleToQuaternion(viewOut, this->Roll , rollQuad);
    NormalizeQuaternion(rollQuad);

    //Get final matrix
    MultiplyQuaternion(rollQuad, this->RestTransform, this->RestTransform);
    NormalizeQuaternion(this->RestTransform);
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildLocalRestPoints()
{
  vtkTransform* transform =
    this->CreatetWorldToBoneParentTransform();
  transform->Inverse();

  double* head = transform->TransformDoublePoint(
    this->GetBoneRepresentation()->GetHeadWorldPosition());
  CopyVector3(head, this->LocalRestHead);

  double* tail = transform->TransformDoublePoint(
    this->GetBoneRepresentation()->GetTailWorldPosition());
  CopyVector3(tail, this->LocalRestTail);

  transform->Delete();
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildLocalPosePoints()
{
  vtkTransform* transform =
    this->CreatetWorldToBoneParentTransform();
  transform->Inverse();

  double* head = transform->TransformDoublePoint(
    this->GetBoneRepresentation()->GetHeadWorldPosition());
  CopyVector3(head, this->LocalPoseHead);

  double* tail = transform->TransformDoublePoint(
    this->GetBoneRepresentation()->GetTailWorldPosition());
  CopyVector3(tail, this->LocalPoseTail);

  transform->Delete();
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
      if (this->HeadWidget)
        {
        this->HeadWidget->GetRepresentation()->SetVisibility(0);
        }
      if (this->TailWidget)
        {
        this->TailWidget->GetRepresentation()->SetVisibility(0);
        }
      }
    else
      {
      if (this->WidgetRep)
        {
        this->WidgetRep->SetVisibility(1);
        }
      if (this->HeadWidget)
        {
        this->HeadWidget->GetRepresentation()->SetVisibility(1);
        }
      if (this->TailWidget)
        {
        this->TailWidget->GetRepresentation()->SetVisibility(1);
        }
      // The interactor must be set prior to enabling the widget.
      if (this->Interactor)
        {
        this->HeadWidget->SetInteractor(this->Interactor);
        this->TailWidget->SetInteractor(this->Interactor);

        this->ParentageLink->SetInteractor(this->Interactor);
        }

      this->HeadWidget->SetEnabled(1);
      this->TailWidget->SetEnabled(1);
      }

    if (this->HeadWidget)
      {
      this->HeadWidget->SetRepresentation(
        vtkBoneRepresentation::SafeDownCast
        (this->WidgetRep)->GetHeadRepresentation());
      this->HeadWidget->SetInteractor(this->Interactor);
      this->HeadWidget->GetRepresentation()->SetRenderer(
        this->CurrentRenderer);
      }
    if (this->TailWidget)
      {
      this->TailWidget->SetRepresentation(
        vtkBoneRepresentation::SafeDownCast
        (this->WidgetRep)->GetTailRepresentation());
      this->TailWidget->SetInteractor(this->Interactor);
      this->TailWidget->GetRepresentation()->SetRenderer(
        this->CurrentRenderer);
      }
    }
  else //disabling widget
    {
    if (this->HeadWidget)
      {
      this->HeadWidget->SetEnabled(0);
      }
    if (this->TailWidget)
      {
      this->TailWidget->SetEnabled(0);
      }
    }

  this->Superclass::SetEnabled(enabling);

  // Add/Remove the actor
  // This needs to be done after enabling the superclass
  // otherwise there isn't a renderer ready.
  if (this->CurrentRenderer)
    {
    if (enabling)
      {
      this->CurrentRenderer->AddActor(this->AxesActor);
      this->UpdateAxesVisibility();
      }
    else
      {
      this->CurrentRenderer->RemoveActor(this->AxesActor);
      this->SetAxesVisibility(vtkBoneWidget::Nothing);
      }
    this->RebuildAxes();
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetRepresentation(vtkBoneRepresentation* r)
{
  if (this->WidgetState == vtkBoneWidget::Pose
    || this->WidgetState == vtkBoneWidget::Rest)
    {
    r->SetHeadWorldPosition(
      this->GetBoneRepresentation()->GetHeadWorldPosition());
    r->SetTailWorldPosition(
      this->GetBoneRepresentation()->GetTailWorldPosition());
    }
  else if (this->WidgetState == vtkBoneWidget::Define)
    {
    r->SetHeadWorldPosition(
      this->GetBoneRepresentation()->GetHeadWorldPosition());
    }

  this->Superclass::SetWidgetRepresentation(
    reinterpret_cast<vtkWidgetRepresentation*>(r));
}

//----------------------------------------------------------------------
vtkBoneRepresentation* vtkBoneWidget::GetBoneRepresentation()
{
  return vtkBoneRepresentation::SafeDownCast(this->WidgetRep);
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

    if (self->HeadLinkedToParent && self->BoneParent)
      {
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetHeadWorldPosition(
        self->BoneParent->GetBoneRepresentation()->GetHeadWorldPosition());
      }
    else
      {
      //Place Point yourself
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetHeadDisplayPosition(e);
      self->HeadWidget->SetEnabled(1);
      }
    }

  // If defining we are placing the second or third point
  else if ( self->WidgetState == vtkBoneWidget::Define )
    {
    //Place Point
    self->WidgetState = vtkBoneWidget::Rest;

    vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->SetTailDisplayPosition(e);
    self->TailWidget->SetEnabled(1);
    self->TailWidget->GetRepresentation()->SetVisibility(1);
    self->WidgetRep->SetVisibility(1);

    self->RebuildRestTransform();
    self->RebuildLocalRestPoints();
    self->RebuildAxes();
    self->RebuildParentageLink();
    }

  else if ( self->WidgetState == vtkBoneWidget::Rest 
            || self->WidgetState == vtkBoneWidget::Pose )
    {
    self->BoneSelected = 0;
    self->HeadSelected = 0;
    self->TailSelected = 0;

    int modifier = self->Interactor->GetShiftKey() | self->Interactor->GetControlKey();
    int state = self->WidgetRep->ComputeInteractionState(X,Y,modifier);
    if ( state == vtkBoneRepresentation::Outside )
      {
      return;
      }

    self->GrabFocus(self->EventCallbackCommand);
    if ( state == vtkBoneRepresentation::OnP1 )
      {
      self->GetBoneRepresentation()->GetHeadRepresentation()->Highlight(1);
      self->HeadSelected = 1;
      self->InvokeEvent(vtkCommand::LeftButtonPressEvent,NULL);
      }
    else if (state == vtkBoneRepresentation::OnP2 )
      {
      self->GetBoneRepresentation()->GetTailRepresentation()->Highlight(1);
      self->TailSelected = 1;
      self->InvokeEvent(vtkCommand::LeftButtonPressEvent,NULL);
      }
    else if ( state == vtkBoneRepresentation::OnLine)
      {
      if (self->WidgetState == vtkBoneWidget::Rest
          || !self->BoneParent)
        {
        self->GetBoneRepresentation()->GetLineHandleRepresentation()->Highlight(1);
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
    // \todo: factorize call to InteractionEvent and SetAbortFlag
    self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
    self->EventCallbackCommand->SetAbortFlag(1);
    }

  else if (self->WidgetState == vtkBoneWidget::Rest)
    {
    if ( self->HeadSelected )
      {
      self->GetBoneRepresentation()->SetHeadDisplayPosition(e);
      self->RebuildRestTransform();
      self->RebuildLocalRestPoints();
      self->RebuildAxes();
      self->RebuildParentageLink();

      // \todo: factorize call to InteractionEvent and SetAbortFlag
      //self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }

    else if ( self->TailSelected )
      {
      self->GetBoneRepresentation()->SetTailDisplayPosition(e);
      self->RebuildRestTransform();
      self->RebuildLocalRestPoints();
      self->RebuildAxes();
      self->RebuildParentageLink();

      self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }

    else if ( self->BoneSelected )
      {
      self->GetBoneRepresentation()->GetLineHandleRepresentation()->SetDisplayPosition(e);
      vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->WidgetInteraction(e);
      self->RebuildRestTransform();
      self->RebuildLocalRestPoints();
      self->RebuildAxes();
      self->RebuildParentageLink();

      if (self->HeadLinkedToParent && self->BoneParent)
        {
        self->BoneParent->LinkTailToChild(self);
        }

      self->InvokeEvent(vtkBoneWidget::RestChangedEvent, NULL);
      self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
      }
    }

  else if (self->WidgetState == vtkBoneWidget::Pose)
    {
    //Cannot move Head in pose mode

    if ( self->TailSelected )
      {
      //
      //Make rotation in camera view plane center on Head
      //

      // Get display positions
      double e1[2], e2[2];
      self->GetBoneRepresentation()->GetHeadDisplayPosition(e1);
      self->GetBoneRepresentation()->GetTailDisplayPosition(e2);

      // Get the current line -> the line between Head and the event
      //in display coordinates
      double currentLine[2], oldLine[2];
      currentLine[0] = e[0] - e1[0];currentLine[1] = e[1] - e1[1];
      vtkMath::Normalize2D(currentLine);

      // Get the old line -> the line between Head and the LAST event
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
      double head[3], tail[3];
      self->GetBoneRepresentation()->GetHeadWorldPosition(head);
      self->GetBoneRepresentation()->GetTailWorldPosition(tail);

      //Get the camera vector
      double cameraVec[3];
      if (!self->GetCurrentRenderer()
          || !self->GetCurrentRenderer()->GetActiveCamera())
        {
        vtkErrorWithObjectMacro(self, "There should be a renderer and a camera."
                                      " Make sure to set these !"
                                      "\n ->Cannot move Tail in pose mode");
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

      //Finally rotate Tail
      vtkSmartPointer<vtkTransform> T = vtkSmartPointer<vtkTransform>::New();
      T->Translate(head); //last transform: Translate back to Head origin
      T->RotateWXYZ(angle, cameraVec); //middle transform: rotate
      //first transform: Translate to world origin
      double minusHead[3];
      CopyVector3(head, minusHead);
      vtkMath::MultiplyScalar(minusHead, -1.0);
      T->Translate(minusHead);

      self->GetBoneRepresentation()->SetTailWorldPosition(
        T->TransformDoublePoint(tail));

      self->RebuildPoseTransform();
      self->RebuildLocalPosePoints();
      self->RebuildAxes();
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
        self->GetBoneRepresentation()
          ->GetLineHandleRepresentation()->SetDisplayPosition(e);
        vtkBoneRepresentation::SafeDownCast(self->WidgetRep)->WidgetInteraction(e);

        self->RebuildPoseTransform();
        self->RebuildLocalPosePoints();
        self->RebuildAxes();
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
  self->HeadSelected = 0;
  self->TailSelected = 0;
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
  CopyQuaternion(this->PoseTransform, this->StartPoseTransform);

  //And update the pose point
  this->GetBoneRepresentation()->GetHeadWorldPosition(this->InteractionWorldHead);
  this->GetBoneRepresentation()->GetTailWorldPosition(this->InteractionWorldTail);

  this->InvokeEvent(vtkBoneWidget::PoseInteractionStoppedEvent, NULL);
}

//-------------------------------------------------------------------------
void vtkBoneWidget::BoneParentPoseChanged()
{
  vtkTransform* transform =
    this->CreatetWorldToBoneParentTransform();

  double* newHead = transform->TransformDoublePoint(this->LocalPoseHead);
  this->GetBoneRepresentation()->SetHeadWorldPosition(newHead);

  double* newTail = transform->TransformDoublePoint(this->LocalPoseTail);
  this->GetBoneRepresentation()->SetTailWorldPosition(newTail);

  transform->Delete();

  this->RebuildPoseTransform();
  this->RebuildAxes();
  this->RebuildParentageLink();
  this->InvokeEvent(vtkBoneWidget::PoseChangedEvent, NULL);
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

    double head[3], tail[3], previousLineVect[3], newLineVect[3], rotationAxis[3], poseAngle;
    // 1- Get Head, Tail, Old Tail
    this->GetBoneRepresentation()->GetHeadWorldPosition( head );
    this->GetBoneRepresentation()->GetTailWorldPosition( tail );

    // 2- Get the previous line directionnal vector
    vtkMath::Subtract(this->InteractionWorldTail, this->InteractionWorldHead, previousLineVect);
    vtkMath::Normalize(previousLineVect);

    // 3- Get the new line vector
    vtkMath::Subtract(tail, head, newLineVect);
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
    // pose mode. The previous transform are stored in StartPoseTransform
    double quad[4];
    AxisAngleToQuaternion(rotationAxis, poseAngle, quad);
    NormalizeQuaternion(quad);
    MultiplyQuaternion(quad, this->StartPoseTransform, this->PoseTransform);
    NormalizeQuaternion(this->PoseTransform);
    }

}

//-------------------------------------------------------------------------
void vtkBoneWidget::BoneParentRestChanged()
{
  this->RebuildLocalRestPoints();

  //In the previous behavior, we had the child Head to follow the parent
  //Tail in distance

  //Now they either are stuck together or nothing
  if (this->HeadLinkedToParent)
    {
    this->LinkHeadToParent();
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

  this->HeadWidget->SetProcessEvents(pe);
  this->TailWidget->SetProcessEvents(pe);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetState(int state)
{
  if ( state == this->WidgetState )
    {
    return;
    }

  switch (state)
    {
    case vtkBoneWidget::Start:
      {
      vtkErrorMacro("Cannot set bone widget to start. The start mode is"
                    " automatically defined when the bone is created,"
                    " but cannot be set by the user.");
      return;
      }
    case vtkBoneWidget::Define:
      {
      vtkErrorMacro("Cannot set bone widget to define. The define mode is"
                    " automatically after the first point has been placed,"
                    " but cannot be set by the user.");
      return;
      }
    case vtkBoneWidget::Rest:
      {
      //Update selection
      this->BoneSelected = 0;
      this->HeadSelected = 0;
      this->TailSelected = 0;

      //Initialize transform and positions
      InitializeQuaternion(this->PoseTransform);
      InitializeQuaternion(this->StartPoseTransform);
      InitializeVector3(this->InteractionWorldHead);
      InitializeVector3(this->InteractionWorldTail);
      InitializeVector3(this->LocalPoseHead);
      InitializeVector3(this->LocalPoseTail);

      if (this->WidgetState != vtkBoneWidget::Pose)
        {
        //Just need to Rebuild transform and local points
        this->RebuildRestTransform();
        this->RebuildLocalRestPoints();
        }
      else // previous state was pose
        {
        //We need to reset the points to their original rest position
        vtkTransform* transform = this->CreatetWorldToBoneParentTransform();

        double* newHead = transform->TransformDoublePoint(this->LocalRestHead);
        this->GetBoneRepresentation()->SetHeadWorldPosition(newHead);

        double* newTail = transform->TransformDoublePoint(this->LocalRestTail);
        this->GetBoneRepresentation()->SetTailWorldPosition(newTail);

        transform->Delete();
        }

      //Update ohers
      this->UpdateAxesVisibility();
      this->RebuildParentageLink();

      break;
      }
    case vtkBoneWidget::Pose:
      {
      //Update selection
      this->BoneSelected = 0;
      this->HeadSelected = 0;
      this->TailSelected = 0;

      //Update local pose
      CopyVector3(this->LocalRestHead, this->LocalPoseHead);
      CopyVector3(this->LocalRestTail, this->LocalPoseTail);
      InitializeQuaternion(this->StartPoseTransform);

      //Update intercation position
      this->GetBoneRepresentation()->GetHeadWorldPosition(this->InteractionWorldHead);
      this->GetBoneRepresentation()->GetTailWorldPosition(this->InteractionWorldTail);

      //Update/Rebuild variables
      if (this->WidgetState != vtkBoneWidget::Rest)
        //this would be a weird case but one never knows
        {
        this->RebuildRestTransform();
        }
      this->RebuildPoseTransform();

      this->UpdateAxesVisibility();
      this->RebuildParentageLink();

      break;
      }
    default:
      {
      vtkErrorMacro("Unknown state. The only possible values are:"
                     "\n    0 <-> Start"
                     "\n    2 <-> Rest"
                     "\n    3 <-> Pose"
                     "\n -> Doing nothing.");
      return;
      }
    }

  this->WidgetState = state;
  this->Modified();
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetStateToPose()
{
  this->SetWidgetState(vtkBoneWidget::Pose);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetWidgetStateToRest()
{
  this->SetWidgetState(vtkBoneWidget::Rest);
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetAxesVisibility(int visibility)
{
  if (this->AxesVisibility == visibility)
    {
    return;
    }

  this->AxesVisibility = visibility;
  this->UpdateAxesVisibility();
}

//----------------------------------------------------------------------
void vtkBoneWidget::UpdateAxesVisibility()
{
  if (this->AxesVisibility == vtkBoneWidget::Nothing
      || this->WidgetState == vtkBoneWidget::Start
      || this->WidgetState == vtkBoneWidget::Define)
    {
    this->AxesActor->SetVisibility(0);
    }
  else
    {
    this->AxesActor->SetVisibility(1);
    }

  this->RebuildAxes();
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildParentageLink()
{
  vtkLineRepresentation* rep=
      vtkLineRepresentation::SafeDownCast(
        this->ParentageLink->GetRepresentation());

  if (this->ShowParentage && this->BoneParent && ! this->HeadLinkedToParent
      && (this->WidgetState == vtkBoneWidget::Rest
          || this->WidgetState == vtkBoneWidget::Pose))
    {
    rep->SetVisibility(1);

    rep->SetPoint1WorldPosition(
      this->BoneParent->GetBoneRepresentation()->GetTailWorldPosition());
    rep->SetPoint2WorldPosition(
      this->GetBoneRepresentation()->GetHeadWorldPosition());
    }
  else
    {
    rep->SetVisibility(0);
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::RebuildAxes()
{
  // only update axes if they are visible to prevent unecessary computation
  if (this->AxesActor->GetVisibility())
    {
    double distance =
      this->GetBoneRepresentation()->GetDistance() * this->AxesSize;
    this->AxesActor->SetTotalLength(distance, distance, distance);

    double axis[3], angle;
    if (this->AxesVisibility == vtkBoneWidget::ShowRestTransform)
      {
      angle = vtkBoneWidget::QuaternionToAxisAngle(this->RestTransform, axis);
      }
    else if (this->AxesVisibility == vtkBoneWidget::ShowPoseTransform)
      {
      angle = vtkBoneWidget::QuaternionToAxisAngle(this->PoseTransform, axis);
      }
    else if (this->AxesVisibility == vtkBoneWidget::ShowPoseTransformAndRestTransform)
      {
      double resultTransform[4];
      MultiplyQuaternion(this->GetPoseTransform(),
                         this->GetRestTransform(),
                          resultTransform);
      NormalizeQuaternion(resultTransform);

      angle = vtkBoneWidget::QuaternionToAxisAngle(resultTransform, axis);
      }

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate( this->GetBoneRepresentation()->GetTailWorldPosition() );
    transform->RotateWXYZ( vtkMath::DegreesFromRadians(angle), axis );

    this->AxesActor->SetUserTransform(transform);
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetHeadLinkedToParent(int link)
{
  if (link == this->HeadLinkedToParent)
    {
    return;
    }

  if (link)
    {
    this->HeadSelected = 0;
    this->LinkHeadToParent();
    }

  this->HeadLinkedToParent = link;

  vtkLineRepresentation* rep=
    vtkLineRepresentation::SafeDownCast(
      this->ParentageLink->GetRepresentation());
  rep->SetVisibility(!link && this->ShowParentage);
  this->ParentageLink->SetEnabled(!link);
}

//----------------------------------------------------------------------
void vtkBoneWidget::LinkHeadToParent()
{
  assert(this->HeadLinkedToParent);

  //Move this Head to Follow the parent movement
  if (this->BoneParent)
    {
    this->SetHeadWorldPosition(
      this->BoneParent->GetBoneRepresentation()->GetTailWorldPosition());
    }
  else
    {
    vtkErrorMacro("Cannot link Head to a non-existing parent."
                  " Set the bone parent before. ->Doing nothing.");
    return;
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::LinkTailToChild(vtkBoneWidget* child)
{
  assert(child);
  // Move this point to snap to given child
  if (child->GetHeadLinkedToParent()) //never too sure
    //(I could even verify the child's parent is indeed this)
    {
    this->SetTailWorldPosition(
      child->GetBoneRepresentation()->GetHeadWorldPosition());
    }
}

//----------------------------------------------------------------------
void vtkBoneWidget::SetShowParentage(int parentage)
{
  if (this->HeadLinkedToParent == parentage)
    {
    return;
    }
  this->ShowParentage = parentage;
  vtkLineRepresentation* rep=
    vtkLineRepresentation::SafeDownCast(
      this->ParentageLink->GetRepresentation());
  rep->SetVisibility(!this->HeadLinkedToParent && this->ShowParentage);
}

//----------------------------------------------------------------------
vtkTransform* vtkBoneWidget::CreatetWorldToBoneParentTransform()
{
  vtkTransform* transform = vtkTransform::New();
  if (!this->BoneParent)
    {
    return transform;
    }

  transform->Translate(this->BoneParent->GetTailWorldPosition());

  double resultTransform[4];
  if (this->WidgetState == Rest)
    {
    CopyQuaternion(this->BoneParent->RestTransform, resultTransform);
    }
  else
    {
    MultiplyQuaternion(this->BoneParent->GetPoseTransform(),
                       this->BoneParent->GetRestTransform(),
                       resultTransform);
    NormalizeQuaternion(resultTransform);
    }

  double axis[3];
  double angle = QuaternionToAxisAngle(resultTransform, axis);
  transform->RotateWXYZ(angle, axis);
  return transform;
}

//----------------------------------------------------------------------
vtkTransform* vtkBoneWidget::CreateWorldToBoneTransform()
{
  double origin[3];
  this->GetBoneRepresentation()->GetHeadWorldPosition(origin);

  vtkTransform* transform = vtkTransform::New();
  transform->Translate( origin );

  double resultTransform[4];
  if (this->WidgetState == Rest)
    {
    CopyQuaternion(this->RestTransform, resultTransform);
    }
  else //Pose mode
    {
    MultiplyQuaternion(this->GetPoseTransform(),
                       this->GetRestTransform(),
                       resultTransform);
    NormalizeQuaternion(resultTransform);
    }

  double axis[3];
  double angle = QuaternionToAxisAngle(resultTransform, axis);
  transform->RotateWXYZ(angle, axis);
  return transform;
}

//----------------------------------------------------------------------
double vtkBoneWidget::QuaternionToAxisAngle(double quad[4], double axis[3])
{
  double angle = acos(quad[0]) * 2.0;
  double f = sin( angle * 0.5 );
  if (f > 1e-13)
    {
    axis[0] = quad[1] / f;
    axis[1] = quad[2] / f;
    axis[2] = quad[3] / f;
    }
  else if (angle > 1e-13 || angle < -1e-13) //means rotation of pi
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

  return angle;
}

//----------------------------------------------------------------------
void vtkBoneWidget::AxisAngleToQuaternion(double axis[3], double angle, double quad[4])
{
  quad[0] = cos( angle / 2.0 );

  double vec[3];
  vec[0] = axis[0];
  vec[1] = axis[1];
  vec[2] = axis[2];
  vtkMath::Normalize(vec);

  double f = sin( angle / 2.0);
  quad[1] =  vec[0] * f;
  quad[2] =  vec[1] * f;
  quad[3] =  vec[2] * f;
}

//----------------------------------------------------------------------
void vtkBoneWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "Bone Widget " << this << "\n";

  os << indent << "Widget State: "<< this->WidgetState<< "\n";

  os << indent << "Selected:"<< "\n";
  os << indent << "  Bone Selected: "<< this->BoneSelected<< "\n";
  os << indent << "  Head Selected: "<< this->HeadSelected<< "\n";
  os << indent << "  Tail Selected: "<< this->TailSelected<< "\n";

  if (this->BoneParent)
    {
    os << indent << "Bone Parent: "<< this->BoneParent << "\n";
    }

  os << indent << "Local Points:" << "\n";
  os << indent << "  Local Rest Head: "<< this->LocalRestHead[0]
                                << "  " << this->LocalRestHead[1]
                                << "  " << this->LocalRestHead[2]<< "\n";
  os << indent << "  Local Rest Tail: "<< this->LocalRestTail[0]
                                << "  " << this->LocalRestTail[1]
                                << "  " << this->LocalRestTail[2]<< "\n";
  os << indent << "  Local Pose Head: "<< this->LocalPoseHead[0]
                                << "  " << this->LocalPoseHead[1]
                                << "  " << this->LocalPoseHead[2]<< "\n";
  os << indent << "  Local Pose Tail: "<< this->LocalPoseTail[0]
                                << "  " << this->LocalPoseTail[1]
                                << "  " << this->LocalPoseTail[2]<< "\n";

  os << indent << "Temporary Points:" << "\n";
  os << indent << "  Interaction World Head: "<< this->InteractionWorldHead[0]
                                << "  " << this->InteractionWorldHead[1]
                                << "  " << this->InteractionWorldHead[2]<< "\n";
  os << indent << "  Interaction World Tail: "<< this->InteractionWorldTail[0]
                                << "  " << this->InteractionWorldTail[1]
                                << "  " << this->InteractionWorldTail[2]<< "\n";

  os << indent << "Tranforms:" << "\n";
  os << indent << "  Rest Transform: "<< this->RestTransform[0]
                                << "  " << this->RestTransform[1]
                                << "  " << this->RestTransform[2]
                                << "  " << this->RestTransform[3]<< "\n";
  os << indent << "  Pose Transform: "<< this->PoseTransform[0]
                                << "  " << this->PoseTransform[1]
                                << "  " << this->PoseTransform[2]
                                << "  " << this->PoseTransform[3]<< "\n";
  os << indent << "  Start PoseTransform: "<< this->StartPoseTransform[0]
                                << "  " << this->StartPoseTransform[1]
                                << "  " << this->StartPoseTransform[2]
                                << "  " << this->StartPoseTransform[3]<< "\n";

  os << indent << "Roll: "<< this->Roll << "\n";

  os << indent << "Parent link: "<< "\n";
  os << indent << "  HeadLinkToParent: "<< this->HeadLinkedToParent << "\n";
  os << indent << "  ShowParentage: "<< this->ShowParentage << "\n";

  os << indent << "Axes:" << "\n";
  os << indent << "  Axes Actor: "<< this->AxesActor << "\n";
  os << indent << "  Axes Visibility: "<< this->AxesVisibility << "\n";
  os << indent << "  Axes Size: "<< this->AxesSize << "\n";
}
