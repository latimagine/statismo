#
# CMakeTargetPath.cmake - Statismo cmake utility module to find target location
#
# This file provides utility to find path of targets on the system.
# Its main usage is for windows system where dll location must be
# passed to the environment PATH.
#

# get list of module/shared directories from list of targets
function(get_shared_locations RES TARGET_LIST)
    foreach(tgt ${TARGET_LIST})
        if(TARGET ${tgt})
            get_target_property(_type ${tgt} TYPE)
            if (${_type} STREQUAL "SHARED_LIBRARY")
               get_target_property(_loc ${tgt} LOCATION)
               get_filename_component(_loc_dir ${_loc} DIRECTORY)
               if(WIN32)
                 file(TO_NATIVE_PATH "${_loc_dir}" _loc_dir)
               endif()
               list(APPEND _res_list ${_loc_dir})
            endif()
        endif()
    endforeach()
    list(REMOVE_DUPLICATES _res_list)
    set(${RES} ${_res_list} PARENT_SCOPE)
endfunction()
