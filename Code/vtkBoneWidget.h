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

#ifndef __vtkBoneWidget_h
#define __vtkBoneWidget_h

// A bone is always define with respect to a frame.
// It can be World or the parent's frame.

// In a parent's frame, x is always along the line, from point 1 to point 2

#include "vtkAbstractWidget.h"
#include "vtkBoneWidgetHeader.h"

#include "vtkCommand.h"

#include <vector>

class vtkBoneRepresentation;
class vtkBoneWidgetCallback;
class vtkHandleWidget;
class vtkLineWidget2;
class vtkPolyDataMapper;
class vtkTransform;

class VTK_BONEWIDGETS_EXPORT vtkBoneWidget : public vtkAbstractWidget
{
public:
  // Description:
  // Instantiate this class.
  static vtkBoneWidget *New();

  // Description:
  // Standard methods for a VTK class.
  vtkTypeMacro(vtkBoneWidget, vtkAbstractWidget);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // The method for activiating and deactiviating this widget. This method
  // must be overridden because it is a composite widget and does more than
  // its superclasses' vtkAbstractWidget::SetEnabled() method.
  virtual void SetEnabled(int);

  // Description:
  void SetRepresentation(vtkBoneRepresentation* r);

  // Description:
  // Return the representation as a vtkBoneRepresentation.
  vtkBoneRepresentation* GetvtkBoneRepresentation()
    {return reinterpret_cast<vtkBoneRepresentation*>(this->WidgetRep);}

  // Description:
  // Create the default widget representation if one is not set.
  void CreateDefaultRepresentation();

  // Description:
  // A flag indicates whether the bi-dimensional measure is valid. The widget
  // becomes valid after two of the four points are placed.
  int IsMeasureValid();

  //BTX
  // Description:
  // Events.
  enum
  {
  EndWidgetSelectEvent = 10050
  };
  //ETX

  // Description:
  // Methods to change the whether the widget responds to interaction.
  // Overridden to pass the state to component widgets.
  virtual void SetProcessEvents(int);

  // Description:
  //Start Mode:   Define the first point when clicked. Goes then to define mode
  //Define Mode:  Define the second point when clicked. Goes then to rest mode
  //Rest Mode:    The bone can be moved and rescaled. If the bone has Children,
  //              the Children will head will (P1) rescale of they are linked
  //              (See P1LinkedToParent)
  //Pose Mode:    The bone can only be rotated. If the bone has Children, the Children
  //              will rotate accordingly but will stay exactly the same
  //              (NO rescaling)
  //BTX
  enum _WidgetState {Start=0,Define,Rest,Pose};
  //ETX

  // Description:
  //OrientationChangedEvent:      Fired when the bone reconstruct its orientation
  //                              This reconstruction happens in Rest mode only.
  //PoseChangedEvent:             Fired in pose mode when a point has been moved
  //PoseInteractionStoppedEvent:  Fired when the interaction is stopped for
  //                              the children of the bone
  //BTX
  enum BoneWidgetEvent {RestChangedEvent = vtkCommand::UserEvent + 1,
                        PoseChangedEvent,
                        PoseInteractionStoppedEvent};
  //ETX

  //Description
  void SetPoint1WorldPosition(double x, double y, double z);
  void SetPoint1WorldPosition(double p1[3]);

  //Description
  void SetPoint2WorldPosition(double x, double y, double z);
  void SetPoint2WorldPosition(double p2[3]);

  //Descritpion
  static double QuaternionToAxisAngle(double quad[4], double axis[3]);
  static void AxisAngleToQuaternion(double axis[3], double angle, double quad[4]);

  // Description:
  virtual void SetWidgetStateToStart();
  virtual void SetWidgetStateToRest();
  virtual void SetWidgetStateToPose();

  // Description:
  void SetBoneParent(vtkBoneWidget* parent);
  vtkBoneWidget* GetBoneParent();

  //Description
  void GetOrientation (double orientation[4]);
  double* GetOrientation ();

    //Description
  void GetPoseTransform (double poseTransform[4]);
  double* GetPoseTransform ();

  //Description
  //Set/get the roll imposed to the matrix, in radians. 0.0 by default.
  vtkGetMacro(Roll, double);
  vtkSetMacro(Roll, double);

  //Description
  //Set/get if the debug axes are visible or not.
  vtkGetMacro(DebugAxes, int);
  void SetDebugAxes (int debugAxes);

  // Description:
  //Nothing:                           Show nothing
  //ShowOrientation:                  The debug axes will output the orientation axes
  //ShowPoseTransform:                The debug axes will output the pose transform axes
  //ShowPoseTransformAndOrientation:  The debug axes will output the result of the orientation
  //                                  and the pose tranform.
  //BTX
  enum _DebugAxes {Nothing = 0,
                   ShowOrientation,
                   ShowPoseTransform,
                   ShowPoseTransformAndOrientation
                  };
  //ETX

  // Description:
  // Return the current widget state.
  virtual int GetWidgetState()
  {return this->WidgetState;}

  //Description:
  //Get the transform from world to bone coordinates.
  //This transform is:
  //    Rest mode T = Orientation + Translation
  //    Pose mode T = Orientation*PoseTransform + Translation
  //    Start/Define mode T = NULL
  //The user is responsible for deleting the transformed received
  vtkTransform* GetWorldToBoneTransform();

  //Description
  // Set/Get if the bone P1 is linked, i.e merged. with the parent P2
  // When setting this to true, the bone P1 is automatically snapped
  // to the parent P2 and the P1 widget is disabled
  // When setting this to false, nothing visible happen but the P1
  // widget is re-enabled.
  vtkGetMacro(P1LinkedToParent, int);
  void SetP1LinkedToParent (int link);

protected:
  vtkBoneWidget();
  ~vtkBoneWidget();

  // The state of the widget
  int WidgetState;
  int BoneSelected;
  int Point1Selected;
  int Point2Selected;

  // Callback interface to capture events when
  // placing the widget.
  static void AddPointAction(vtkAbstractWidget*);
  static void MoveAction(vtkAbstractWidget*);
  static void EndSelectAction(vtkAbstractWidget*);

  // The positioning handle widgets
  vtkHandleWidget *Point1Widget;
  vtkHandleWidget *Point2Widget;
  vtkBoneWidgetCallback *BoneWidgetCallback1;
  vtkBoneWidgetCallback *BoneWidgetCallback2;

  // Methods invoked when the handles at the
  // end points of the widget are manipulated
  void StartBoneInteraction();
  virtual void EndBoneInteraction();

  //Set the current bone parent.
  vtkBoneWidget*              BoneParent;
  vtkBoneWidgetCallback*      BoneWidgetChildrenCallback;
  double                      LocalP1[3];
  double                      LocalP2[3];
  double                      PoseP1[3];
  double                      PoseP2[3];
  double                      OldPoseTransform[4];
  double                      Roll; // in radians
  double                      Orientation[4];
  double                      PoseTransform[4];
  int                         P1LinkedToParent;

  int                         DebugAxes;
  vtkLineWidget2*             DebugX;
  vtkLineWidget2*             DebugY;
  vtkLineWidget2*             DebugZ;
  double                      DebugAxesSize;

  void RebuildOrientation();
  void RebuildLocalPoints();
  void RebuildPoseTransform();
  void RebuildDebugAxes();

  //Move the point p1 to the parent p2
  void LinkPoint1ToParent();
  //Mmove this P2 to the child's P1. Used for translations.
  void LinkParentPoint2To(vtkBoneWidget* child);

  //Function called upon Parent events
  void BoneParentPoseChanged();
  void BoneParentInteractionStopped();
  void BoneParentRestChanged();

  //void BoneParentOrientationChanged();

//BTX
  friend class vtkBoneWidgetCallback;
//ETX

private:
  vtkBoneWidget(const vtkBoneWidget&);  //Not implemented
  void operator=(const vtkBoneWidget&);  //Not implemented
};

#endif
