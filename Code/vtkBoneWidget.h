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

//TO DO: ADD COMMENTS

// A bone is always defined with respect to a frame.
// It can be World or the parent's frame.
// In a parent's frame, Y is always along the line, from point 1 to point 2.

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
  vtkBoneRepresentation* GetBoneRepresentation();

  // Description:
  // Create the default widget representation (vtkBoneRepresentation) if no one is set.
  virtual void CreateDefaultRepresentation();

  // Description:
  // Methods to change the whether the widget responds to interaction.
  // Overridden to pass the state to component widgets.
  virtual void SetProcessEvents(int process);

  // Description:
  // Start Mode:  Define the first point when clicked. Goes then to define mode.
  // Define Mode:  Define the second point when clicked. Goes then to rest mode.
  // Rest Mode:  The bone can be moved and rescaled. If the bone has Children,
  //             the Children will head will (Head) rescale of they are linked
  //             (See HeadLinkedToParent).
  // Pose Mode:  The bone can only be rotated. If the bone has Children, the Children
  //             will rotate accordingly but will stay exactly the same
  //             (NO rescaling).
  //BTX
  enum WidgetStateType {Start=0,Define,Rest,Pose};
  //ETX

  // Description:
  // RestChangedEvent:  Fired when the bone reconstruct its orientation
  //                    This reconstruction happens in Rest mode only.
  // PoseChangedEvent:  Fired in pose mode when a point has been moved
  // PoseInteractionStoppedEvent:  Fired when the interaction is stopped for
  //                               the children of the bone
  //BTX
  enum BoneWidgetEventType {RestChangedEvent = vtkCommand::UserEvent + 1,
                            PoseChangedEvent,
                            PoseInteractionStoppedEvent};
  //ETX

  // Description:
  // Set/Get the point1 world position
  void SetPoint1WorldPosition(double x, double y, double z);
  void SetPoint1WorldPosition(double Head[3]);
  void GetPoint1WorldPosition(double Head[3]);
  double* GetPoint1WorldPosition();

  // Description:
  //Set/Get the point2 world position
  void SetPoint2WorldPosition(double x, double y, double z);
  void SetPoint2WorldPosition(double Tail[3]);
  void GetPoint2WorldPosition(double Head[3]);
  double* GetPoint2WorldPosition();

  // Descritpion:
  // Helper function for conversion quaternion conversion
  // to and from rotation/axis
  static double QuaternionToAxisAngle(double quad[4], double axis[3]);
  static void AxisAngleToQuaternion(double axis[3], double angle, double quad[4]);

  // Description:
  // Set/Get the widget state.
  // Start Mode:   Define the first point when clicked. Goes then to define mode
  // Define Mode:  Define the second point when clicked. Goes then to rest mode
  // Rest Mode:    The bone can be moved and rescaled. If the bone has Children,
  //               the Children will head will (Head) rescale of they are linked
  //               (See HeadLinkedToParent)
  // Pose Mode:    The bone can only be rotated. If the bone has Children, the Children
  //               will rotate accordingly but will stay exactly the same
  //               (NO rescaling)
  vtkGetMacro(WidgetState, int);
  void SetWidgetState(int state);
  void SetWidgetStateToStart();
  void SetWidgetStateToRest();
  void SetWidgetStateToPose();

  // Description:
  // Get/Set the bone's parent. If NULL, then the bone is considerer like root
  void SetBoneParent(vtkBoneWidget* parent);
  vtkBoneWidget* GetBoneParent();

  // Description:
  // Get/Set the bone's orientation. The orientation is updated in rest mode
  // and fixed in pose mode. It is undefined in the other modes.
  void GetOrientation (double orientation[4]);
  double* GetOrientation ();

  //Description
  //Get/Set the bone's pose transform. The pose transform is updated in pose
  //mode. It is undefined in the other modes.
  void GetPoseTransform (double poseTransform[4]);
  double* GetPoseTransform ();

  //Description
  //Set/get the roll imposed to the matrix, in radians. 0.0 by default.
  vtkGetMacro(Roll, double);
  vtkSetMacro(Roll, double);

  //Description
  //Set/get if the debug axes are visible or not.
  //Nothing <-> 0:                          Show nothing
  //ShowOrientation <-> 1:                  The debug axes will output the
  //                                        orientation axes
  //ShowPoseTransform  <-> 2:               The debug axes will output the
  //                                        pose transform axes
  //ShowPoseTransformAndOrientation <-> 3:  The debug axes will output the
  //                                        result of the orientation
  //                                        and the pose tranform.
  vtkGetMacro(DebugAxes, int);
  void SetDebugAxes (int debugAxes);

  // Description:
  //Nothing:                          Show nothing
  //ShowOrientation:                  The debug axes will output the orientation axes
  //ShowPoseTransform:                The debug axes will output the pose transform axes
  //ShowPoseTransformAndOrientation:  The debug axes will output the result of the orientation
  //                                  and the pose tranform.
  //BTX
  enum DebugAxesType {Nothing = 0,
                      ShowOrientation,
                      ShowPoseTransform,
                      ShowPoseTransformAndOrientation
                      };
  //ETX

  // Description:
  // Get the transform from world to bone coordinates.
  // This transform is:
  //    Rest mode T = Orientation + Translation
  //    Pose mode T = Orientation*PoseTransform + Translation
  //    Start/Define mode T = NULL
  // The user is responsible for deleting the transformed received.
  vtkTransform* CreateWorldToBoneTransform();

  //Description
  // Set/Get if the bone Head is linked, i.e merged. with the parent Tail
  // When setting this to true, the bone Head is automatically snapped
  // to the parent Tail and the Head widget is disabled
  // When setting this to false, nothing visible happen but the Head
  // widget is re-enabled.
  vtkGetMacro(HeadLinkedToParent, int);
  void SetHeadLinkedToParent (int link);

  //Description
  // Show/Hide the link between a child an its parent
  vtkGetMacro(ShowParentage, int);
  void SetShowParentage (int parentage);

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

  //Bone widget essentials
  vtkBoneWidget*              BoneParent;
  vtkBoneWidgetCallback*      BoneWidgetChildrenCallback;
  double                      LocalRestHead[3];
  double                      LocalRestTail[3];
  double                      LocalPoseHead[3];
  double                      LocalPoseTail[3];
  double                      TemporaryPoseHead[3];
  double                      TemporaryPoseTail[3];
  double                      OldPoseTransform[4];
  double                      Roll; // in radians
  double                      Orientation[4];
  double                      PoseTransform[4];

  //For the link between parent and child
  int                         HeadLinkedToParent;
  int                         ShowParentage;
  vtkLineWidget2*             ParentageLink;

  //For an easier debug
  int                         DebugAxes;
  vtkLineWidget2*             DebugX;
  vtkLineWidget2*             DebugY;
  vtkLineWidget2*             DebugZ;
  double                      DebugAxesSize;

  //Essentials functions
  void RebuildOrientation();
  void RebuildLocalRestPoints();
  void RebuildLocalPosePoints();
  void RebuildPoseTransform();
  void RebuildDebugAxes();
  void RebuildParentageLink();

  // Description:
  // Move the point Head to the parent Tail
  void LinkHeadToParent();
  // Description:
  // Move this Tail to the child's Head. Used for translations.
  void LinkTailToChild(vtkBoneWidget* child);

  //Function called upon Parent events
  void BoneParentPoseChanged();
  void BoneParentInteractionStopped();
  void BoneParentRestChanged();

//BTX
  friend class vtkBoneWidgetCallback;
//ETX

private:
  vtkBoneWidget(const vtkBoneWidget&);  //Not implemented
  void operator=(const vtkBoneWidget&);  //Not implemented
};

#endif
