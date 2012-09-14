#
##### Testing #####
#

create_test_sourcelist (BoneWidgetTest_Sources
                         vtkBoneWidgetTests.cxx
                         vtkBoneWidgetRepresentationAndInteractionTest.cxx
                         vtkBoneWidgetTwoBonesTest.cxx
                         vtkBoneWidgetThreeBonesTest.cxx
                         vtkBoneWidgetTwoBonesTestRotationMatrix.cxx
                        )                       

add_executable (vtkBoneWidgetTests ${BoneWidgetTest_Sources})
target_link_libraries(vtkBoneWidgetTests ${VTK_LIBRARIES} vtkBoneWidget)

add_test(vtkBoneWidgetRepresentationAndIntercationTest ${CXX_TEST_PATH}/BoneWidgetTests vtkBoneWidgetRepresentationAndIntercationTest)

add_test(vtkBoneWidgetTwoBonesTest ${CXX_TEST_PATH}/BoneWidgetTests vtkBoneWidgetTwoBonesTest)

add_test(vtkBoneWidgetThreeBonesTest ${CXX_TEST_PATH}/BoneWidgetTests vtkBoneWidgetThreeBonesTest)

add_test(vtkBoneWidgetTwoBonesTest ${CXX_TEST_PATH}/BoneWidgetTests vtkBoneWidgetTwoBonesTestRotationMatrix)


