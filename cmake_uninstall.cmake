# CMake uninstall script
# Based on CMake's install_manifest.txt

if(NOT EXISTS "/home/ligx/workspace/openarm/openarm_can/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: /home/ligx/workspace/openarm/openarm_can/install_manifest.txt")
endif()

file(READ "/home/ligx/workspace/openarm/openarm_can/install_manifest.txt" _manifest)
string(REPLACE "\n" ";" _manifest_list "${_manifest}")

foreach(_file IN LISTS _manifest_list)
  if(_file STREQUAL "")
    continue()
  endif()

  if(EXISTS "${_file}" OR IS_SYMLINK "${_file}")
    message(STATUS "Uninstalling: ${_file}")
    file(REMOVE "${_file}")
    if(EXISTS "${_file}" OR IS_SYMLINK "${_file}")
      message(FATAL_ERROR "Failed to remove: ${_file}")
    endif()
  else()
    message(STATUS "Already removed: ${_file}")
  endif()
endforeach()
