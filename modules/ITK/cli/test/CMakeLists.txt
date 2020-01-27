# Test some of the commands provided in the man pages
# Not all commands for 3D deformation models will be tested because the example image data is in 2d
set(_test_deformation_data_dir "${CMAKE_BINARY_DIR}/tests-cli/deformation/")
set(_test_shape_data_dir "${CMAKE_BINARY_DIR}/tests-cli/shape/")
set(_statismo_data_dir "${statismo_SOURCE_DIR}/data/")

# write a file containing the paths to the deformation fields/meshes to build the models.
# in this test the absolute paths are used so that it's independent from the current working directory. Paths relative to the current working directory can be used as well.
function(write_filelist fileName filePattern)
  file(GLOB file_list "${filePattern}")
  if(EXISTS ${fileName})
    FILE(REMOVE ${fileName})
  endif()
  foreach(line ${file_list})
    file(APPEND ${fileName} "${line}\n")
  endforeach()
  foreach(line ${ARGN})
    file(APPEND ${fileName} "${line}\n")
  endforeach()
endfunction()

set(_filename_deformation_field "${_test_deformation_data_dir}data-list.txt")
write_filelist(${_filename_deformation_field} "${_statismo_data_dir}/hand_dfs/df-hand-*.vtk")

# List of 2D tests

set(_filename_deformation_model_2d "${_test_deformation_data_dir}2d deformationmodel.h5")
add_test(NAME statismo-build-deformation-model-test-2d
  COMMAND statismo-build-deformation-model -d 2 -l "${_filename_deformation_field}" "${_filename_deformation_model_2d}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

set(_filename_gaussian_deformation_model_2d "${_test_deformation_data_dir}2d gaussian deformationmodel.h5")
add_test(NAME statismo-build-gp-model-test-2d-image
  COMMAND statismo-build-gp-model -d 2 -t deformation -k gaussian -p 75 -s 100 -n 100 -r "${_statismo_data_dir}hand_images/hand-1.vtk" "${_filename_gaussian_deformation_model_2d}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-build-gp-model-test-2d-image-extend-existing-model
    COMMAND statismo-build-gp-model -d 2 -t deformation -k gaussian -p 75 -s 100 -n 100 -m "${_filename_gaussian_deformation_model_2d}" "${_test_deformation_data_dir}2d extended gaussian deformationmodel.h5"
    WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-gp-model-test-2d-image-extend-existing-model APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-sample-test-2d-image
  COMMAND statismo-sample -d 2 -t deformation -i "${_filename_deformation_model_2d}" "${_test_deformation_data_dir}sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-2d-image APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

set(_filename__deformation_field "${_test_deformation_data_dir}model-deform-field-no-landmarks.vtk")
add_test(NAME statismo-fit-image-test-2d-no-landmarks
  COMMAND statismo-fit-image -d 2 -i "${_filename_gaussian_deformation_model_2d}" -m "${_statismo_data_dir}/hand_images/hand-2.vtk" -w 0.25 -f "${_statismo_data_dir}/hand_images/hand-1.vtk" -e "${_filename__deformation_field}" -a "${_test_deformation_data_dir}entire-deform-field-no-landmarks.vtk" -p "${_test_deformation_data_dir}projection-hand2-no-landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-image-test-2d-no-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-fit-image-test-2d-with-landmarks
  COMMAND statismo-fit-image -d 2 -i "${_filename_gaussian_deformation_model_2d}" -m "${_statismo_data_dir}/hand_images/hand-2.vtk" -w 0.25 -f "${_statismo_data_dir}/hand_images/hand-1.vtk" --fixed-landmarks "${_statismo_data_dir}/hand_landmarks/hand-2.csv" --moving-landmarks "${_statismo_data_dir}/hand_landmarks/hand-1.csv" -v 0.1 -p "${_test_deformation_data_dir}projection-hand2-with-landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-image-test-2d-with-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test-2d-image)

add_test(NAME statismo-reduce-model-test-2d
  COMMAND statismo-reduce-model -d 2 -t deformation -i "${_filename_deformation_model_2d}" -v 91 "${_test_deformation_data_dir}reduced 2D model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-2d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

add_test(NAME statismo-posterior-test-2d
  COMMAND statismo-posterior -d 2 -i "${_filename_deformation_model_2d}" -t deformation -f "${_statismo_data_dir}/hand_landmarks/hand-2.csv" -m "${_statismo_data_dir}/hand_landmarks/hand-1.csv" -v 0.5 "${_test_deformation_data_dir}2D posterior model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-2d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-2d)

add_test(NAME statismo-warp-image-test-2d
  COMMAND statismo-warp-image -d 2 -i "${_statismo_data_dir}/hand_images/hand-1.vtk" -f "${_filename__deformation_field}" "${_test_deformation_data_dir}hand-1 image warped to hand-2.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-warp-image-test-2d APPEND PROPERTY DEPENDS statismo-fit-image-test-2d-no-landmarks)

# List of 3D tests

set(_filename_deformation_model_3d "${_test_deformation_data_dir}3d deformationmodel.h5")
add_test(NAME statismo-build-deformation-model-test-3d
  COMMAND statismo-build-deformation-model -l "${_filename_deformation_field}" "${_filename_deformation_model_3d}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

set(_filename_gaussian_deformation_model_3d "${_test_deformation_data_dir}3d gaussian deformationmodel.h5")
add_test(NAME statismo-build-gp-model-test-3d-image
  COMMAND statismo-build-gp-model -t deformation -k gaussian -p 75 -s 100 -n 25 -r "${_statismo_data_dir}hand_images/hand-0.vtk" "${_filename_gaussian_deformation_model_3d}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-reduce-model-test-3d
  COMMAND statismo-reduce-model -t deformation -i "${_filename_deformation_model_3d}" -v 91 "${_test_deformation_data_dir}reduced 3D model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-3d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-3d)

add_test(NAME statismo-posterior-test-3d
  COMMAND statismo-posterior -i "${_filename_deformation_model_3d}" -t deformation -f "${_statismo_data_dir}/hand_landmarks/hand-2.csv" -m "${_statismo_data_dir}/hand_landmarks/hand-1.csv" -v 0.5 "${_test_deformation_data_dir}3D posterior model.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-3d APPEND PROPERTY DEPENDS statismo-build-deformation-model-test-3d)

set(_filename_3d_deform_sample "${_test_deformation_data_dir}3d deformation field sample.vtk")
add_test(NAME statismo-sample-test-3d-image
  COMMAND statismo-sample -t deformation -i "${_filename_gaussian_deformation_model_3d}" "${_filename_3d_deform_sample}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-3d-image APPEND PROPERTY DEPENDS statismo-build-gp-model-test-3d-image)

add_test(NAME statismo-warp-image-test-3d
  COMMAND statismo-warp-image -i "${_statismo_data_dir}hand_images/hand-0.vtk" -f "${_filename_3d_deform_sample}" "${_test_deformation_data_dir}hand-0 warp.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-warp-image-test-3d APPEND PROPERTY DEPENDS statismo-sample-test-3d-image)

set(_filename_gp_shape_model "${_test_shape_data_dir}gp shapemodel.h5")
add_test(NAME statismo-build-gp-model-test
  COMMAND statismo-build-gp-model -t shape -k gaussian -p 95.5 -s 100.42 -n 150 -r "${_statismo_data_dir}/femur_meshes/VSD001_femur.vtk" "${_filename_gp_shape_model}"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")

add_test(NAME statismo-reduce-model-test-5-components
  COMMAND statismo-reduce-model -i "${_filename_gp_shape_model}" -n 5 "${_test_shape_data_dir}reduced_model 5 components.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-5-components APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-reduce-model-test-42.1-percent
  COMMAND statismo-reduce-model -i "${_filename_gp_shape_model}" -v 42.1 "${_test_shape_data_dir}reduced_model 42.1 percent.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-reduce-model-test-42.1-percent APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-random
  COMMAND statismo-sample -i "${_filename_gp_shape_model}" "${_test_shape_data_dir}random sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-random APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-random-2
  COMMAND statismo-sample -i "${_filename_gp_shape_model}" "${_test_shape_data_dir}random sample 2.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-random-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-mean
  COMMAND statismo-sample -m -i "${_filename_gp_shape_model}" "${_test_shape_data_dir}mean sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-mean APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-reference
  COMMAND statismo-sample -r -i "${_filename_gp_shape_model}" "${_test_shape_data_dir}reference sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-reference APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-sample-test-parameters
  COMMAND statismo-sample -p 1:1,4:0.1,9:2.5 -i "${_filename_gp_shape_model}" "${_test_shape_data_dir}reference sample.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-sample-test-parameters APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-fit-surface-test-no-landmarks
  COMMAND statismo-fit-surface  -i "${_filename_gp_shape_model}" -t "${_statismo_data_dir}/femur_meshes/VSD002_femur.vtk" -w 0.01 -j "${_test_shape_data_dir}projected mesh no landmarks.vtk" -o "${_test_shape_data_dir}fitted mesh no landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-no-landmarks APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

set(_filename_projected_mesh "${_test_shape_data_dir}projected mesh with landmarks.vtk")
add_test(NAME statismo-fit-surface-test-with-landmarks-1
  COMMAND statismo-fit-surface  -i "${_filename_gp_shape_model}" -t "${_statismo_data_dir}/femur_meshes/VSD002_femur.vtk" -w 0.1 -j "${_filename_projected_mesh}" -f "${_statismo_data_dir}/femur_meshes/VSD001-lm.csv"  -m "${_statismo_data_dir}/femur_meshes/VSD002-lm.csv" -v 0.1 
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-with-landmarks-1 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-fit-surface-test-with-landmarks-2
  COMMAND statismo-fit-surface  -i "${_filename_gp_shape_model}" -t "${_statismo_data_dir}/femur_meshes/VSD002_femur.vtk" -w 0.05 -f "${_statismo_data_dir}/femur_meshes/VSD001-lm.csv"  -m "${_statismo_data_dir}/femur_meshes/VSD002-lm.csv" -v 0.1 -p -o "${_test_shape_data_dir}fitted mesh with landmarks.vtk"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-fit-surface-test-with-landmarks-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-1
  COMMAND statismo-posterior  -i "${_filename_gp_shape_model}" -f "${_statismo_data_dir}/femur_meshes/VSD001-lm.csv"  -m "${_statismo_data_dir}/femur_meshes/VSD002-lm.csv" -v 0.1 "${_test_shape_data_dir}posterior gp model 1.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-1 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-2
  COMMAND statismo-posterior  -i "${_filename_gp_shape_model}" -t shape -f "${_statismo_data_dir}/femur_meshes/VSD001-lm.csv"  -m "${_statismo_data_dir}/femur_meshes/VSD002-lm.csv" -v 0.1 "${_test_shape_data_dir}posterior gp model 2.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-2 APPEND PROPERTY DEPENDS statismo-build-gp-model-test)

add_test(NAME statismo-posterior-test-with-corresponding-mesh
  COMMAND statismo-posterior  -i "${_filename_gp_shape_model}" -c "${_filename_projected_mesh}" "${_test_shape_data_dir}posterior gp model with corresponding mesh.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-posterior-test-with-corresponding-mesh APPEND PROPERTY DEPENDS statismo-build-gp-model-test)
set_property(TEST statismo-posterior-test-with-corresponding-mesh APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)

set(_filename_meshes "${_test_shape_data_dir}data-list.txt")
write_filelist(${_filename_meshes} "${_statismo_data_dir}/femur_meshes/VSD001_femur.vtk" "${_filename_projected_mesh}")

add_test(NAME statismo-build-shape-model-test-1
  COMMAND statismo-build-shape-model --data-list "${_filename_meshes}" "${_test_shape_data_dir}shape model 1.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-1 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)

add_test(NAME statismo-build-shape-model-test-2
  COMMAND statismo-build-shape-model -p GPA -l "${_filename_meshes}" "${_test_shape_data_dir}shape model 2.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-2 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)

add_test(NAME statismo-build-shape-model-test-3
  COMMAND statismo-build-shape-model -p reference -r "${_filename_projected_mesh}" -l "${_filename_meshes}" "${_test_shape_data_dir}shape model 3.h5"
  WORKING_DIRECTORY "${TEST_WORKING_DIR}")
set_property(TEST statismo-build-shape-model-test-3 APPEND PROPERTY DEPENDS statismo-fit-surface-test-with-landmarks-1)
