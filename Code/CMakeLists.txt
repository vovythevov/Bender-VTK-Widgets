
set (BoneWidget_Sources
     vtkBoneRepresentation.h
     vtkBoneRepresentation.cxx
     vtkBoneWidget.h
     vtkBoneWidget.cxx
     vtkBoneWidgetHeader.h
     vtkCylinderBoneRepresentation.h
     vtkCylinderBoneRepresentation.cxx
     vtkDoubleConeBoneRepresentation.h
     vtkDoubleConeBoneRepresentation.cxx
     )

add_library (vtkBoneWidget ${LIB_TYPE} ${BoneWidget_Sources})
target_link_libraries(vtkBoneWidget ${VTK_LIBRARIES})
