# remove all matching elements from the list
# originally from OpenCV
macro(list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()

# stable & safe duplicates removal macro
# originally from OpenCV
macro(list_unique __lst)
  if(${__lst})
    list(REMOVE_DUPLICATES ${__lst})
  endif()
endmacro()

# convert list of paths to libraries names without lib prefix
# originally from OpenCV
macro(convert_to_lib_name var)
  set(__tmp "")
  foreach(path ${ARGN})
    get_filename_component(__tmp_name "${path}" NAME_WE)
    string(REGEX REPLACE "^lib" "" __tmp_name ${__tmp_name})
    list(APPEND __tmp "${__tmp_name}")
  endforeach()
  set(${var} ${__tmp})
  unset(__tmp)
  unset(__tmp_name)
endmacro()