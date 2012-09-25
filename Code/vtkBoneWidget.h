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

class vtkAxesActor;
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
  // The modes work following this diagram:
  // Start -> Define -> Rest <-> Pose
  //
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
  // RestChangedEvent:  Fired when the bone reconstruct its RestTransform
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
  // Set/Get the head world position
  // The user should try to be in rest mode when using those methods.
  // If the bone is in pose mode, this will output an error messages.
  // In pose mode, the head cannot be moved. (It will however automatically
  // follows its parent movement)
  void SetHeadWorldPosition(double x, double y, double z);
  void SetHeadWorldPosition(double Head[3]);
  void GetHeadWorldPosition(double Head[3]);
  double* GetHeadWorldPosition();

  // Description:
  // Set/Get the tail world position
  // The user should try to be in rest mode when using those methods.
  // If the bone is in pose mode, this will output an error messages.
  // The rotates methods must be used when moving the tail.
  void SetTailWorldPosition(double x, double y, double z);
  void SetTailWorldPosition(double Tail[3]);
  void GetTailWorldPosition(double Head[3]);
  double* GetTailWorldPosition();

  // Description:
  // Rotation methods to move the tail. Those methods can be used in any modes.
  // Note: In pose mode, those methods must be used instead of
  // SetTailWorldPosition. Angle is in radians.
  void RotateTailX(double angle);
  void RotateTailY(double angle);
  void RotateTailZ(double angle);
  void RotateTailWXYZ(double angle, double x, double y, double z);
  void RotateTailWXYZ(double angle, double axis[3]);

  // Descritption:
  // Helper function for conversion quaternion conversion
  // to and from rotation/axis
  // Angle is in radians
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
  void SetWidgetStateToRest();
  void SetWidgetStateToPose();

  // Description:
  // Get/Set the bone's parent. If NULL, then the bone is considerer like root
  void SetBoneParent(vtkBoneWidget* parent);
  vtkBoneWidget* GetBoneParent();

  // Description:
  // Get/Set the bone's RestTransform. The RestTransform is updated in rest mode
  // and fixed in pose mode. It is undefined in the other modes.
  void GetRestTransform (double restTransform[4]);
  double* GetRestTransform ();

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
  //ShowRestTransform <-> 1:                  The debug axes will output the
  //                                        RestTransform axes
  //ShowPoseTransform  <-> 2:               The debug axes will output the
  //                                        pose transform axes
  //ShowPoseTransformAndRestTransform <-> 3:  The debug axes will output the
  //                                        result of the RestTransform
  //                                        and the pose tranform.
  // The axes labels are disables by defaut. To change rendering properties,
  // see GetAxesActor().
  vtkGetMacro(AxesVisibility, int);
  void SetAxesVisibility (int AxesVisibility);

  // Description:
  //Nothing:                          Show nothing
  //ShowRestTransform:                  The debug axes will output the RestTransform axes
  //ShowPoseTransform:                The debug axes will output the pose transform axes
  //ShowPoseTransformAndRestTransform:  The debug axes will output the result of the RestTransform
  //                                  and the pose tranform.
  //BTX
  enum AxesVisibilityType {Nothing = 0,
                           ShowRestTransform,
                           ShowPoseTransform,
                           ShowPoseTransformAndRestTransform
                          };
  //ETX

  // Description:
  // Get the transform from world to bone coordinates.
  // This transform is:
  //    Rest mode T = RestTransform + Translation
  //    Pose mode T = RestTransform*PoseTransform + Translation
  //    Start/Define mode T = NULL
  // The user is responsible for deleting the transformed received.
  vtkTransform* CreateWorldToBoneTransform();

  // Description:
  // Get the transform from world to bone parent coordinates.
  // This transform is identity is the bone does not have a parent
  // Otherwise:
  //    Rest mode T = BoneParentRestTransform + Translation
  //    Pose mode T = BonreParentRestTransform*BoneParentPoseTransform
  //                  + Translation
  //    Start/Define mode T = NULL
  /// Where the translation is the translation by the bone parent's tail
  // The user is responsible for deleting the transformed received.
  vtkTransform* CreateWorldToBoneParentTransform();

  // Description:
  // Set/Get if the bone Head is linked, i.e merged. with the parent Tail
  // When setting this to true, the bone Head is automatically snapped
  // to the parent Tail and the Head widget is disabled
  // When setting this to false, nothing visible happen but the Head
  // widget is re-enabled.
  vtkGetMacro(HeadLinkedToParent, int);
  void SetHeadLinkedToParent (int link);

  // Description
  // Show/Hide the link between a child an its parent
  vtkGetMacro(ShowParentage, int);
  void SetShowParentage (int parentage);

  // Description:
  // Get the Axes actor. This is meant for the user to
  // modify the rendering properties of the actor. The
  // other properties must be left unchanged.
  vtkAxesActor* GetAxesActor()
  { return this->AxesActor; };

protected:
  vtkBoneWidget();
  ~vtkBoneWidget();

  // The state of the widget
  int WidgetState;
  int BoneSelected;
  int HeadSelected;
  int TailSelected;

  // Callback interface to capture events when
  // placing the widget.
  static void AddPointAction(vtkAbstractWidget*);
  static void MoveAction(vtkAbstractWidget*);
  static void EndSelectAction(vtkAbstractWidget*);

  // The positioning handle widgets
  vtkHandleWidget *HeadWidget;
  vtkHandleWidget *TailWidget;
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
  double                      InteractionWorldHead[3];
  double                      InteractionWorldTail[3];
  double                      StartPoseTransform[4];
  double                      Roll; // in radians
  double                      RestTransform[4];
  double                      PoseTransform[4];

  //For the link between parent and child
  int                         HeadLinkedToParent;
  int                         ShowParentage;
  vtkLineWidget2*             ParentageLink;

  //For an easier debug and understanding
  int                         AxesVisibility;
  vtkAxesActor*               AxesActor;
  double                      AxesSize;

  //Essentials functions
  void RebuildRestTransform();
  void RebuildLocalRestPoints();
  void RebuildLocalPosePoints();
  void RebuildPoseTransform();
  void RebuildAxes();
  void RebuildParentageLink();

  void UpdateParentageLinkVisibility();
  void UpdateAxesVisibility();

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
