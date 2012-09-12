
set (BoneWidget_Sources
     vtkBoneRepresentation.h
     vtkBoneRepresentation.cxx
     vtkBoneWidget.h
     vtkBoneWidget.cxx
     vtkCylinderBoneRepresentation.h
     vtkCylinderBoneRepresentation.cxx
     vtkDoubleConeBoneRepresentation.h
     vtkDoubleConeBoneRepresentation.cxx
     )

add_library (BoneWidget_Libraries  ${BoneWidget_Sources})   