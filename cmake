cmake_minimum_required (VERSION 2.6)

string(REGEX REPLACE "cmake$" "" legit_path ${CMAKE_CURRENT_LIST_FILE})

if (NOT TARGET legit)
  add_subdirectory(${legit_path} legit_build)
endif()
