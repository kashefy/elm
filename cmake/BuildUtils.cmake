# estimate library suffix and prefix and prepend+append to library name
macro(add_lib_prefix_suffix var)
  set(__tmp "")
  foreach(m ${ARGN})
    set(__tmp_name "${m}")

    if(${BUILD_SHARED_LIBS})
        set(__tmp_name_prefix ${CMAKE_SHARED_LIBRARY_PREFIX})
        set(__tmp_name_suffix ${CMAKE_SHARED_LIBRARY_SUFFIX})
    else(${BUILD_SHARED_LIBS})
        set(__tmp_name_prefix ${CMAKE_STATIC_LIBRARY_PREFIX})
        set(__tmp_name_suffix ${CMAKE_STATIC_LIBRARY_SUFFIX})
    endif(${BUILD_SHARED_LIBS})

    list(APPEND __tmp ${__tmp_name_prefix}${__tmp_name}${__tmp_name_suffix})

  endforeach()
  set(${var} ${__tmp})
  unset(__tmp)
  unset(__tmp_name)

endmacro()
