# ==============================================================================
# Purdue ROV (X19-ROV) - Hardware Target Boards
# ==============================================================================

# Dynamically discover and include all board target directories that have a CMakeLists.txt
file(GLOB BOARD_SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(DIR ${BOARD_SUBDIRS})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/CMakeLists.txt")
        add_subdirectory(${DIR})
    endif()
endforeach()
