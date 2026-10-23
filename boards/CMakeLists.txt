# ==============================================================================
# Purdue ROV (X19-ROV) - Hardware Target Boards
# ==============================================================================

# 1. Add standard baseline boards first so their platform libraries are available
set(BASELINE_BOARDS f411_nucleo g474_nucleo stm32c5)
foreach(BASE ${BASELINE_BOARDS})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${BASE}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${BASE}/CMakeLists.txt")
        add_subdirectory(${BASE})
    endif()
endforeach()

# 2. Dynamically discover, scaffold, and include all other custom board directories
file(GLOB BOARD_SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(DIR ${BOARD_SUBDIRS})
    if(NOT DIR IN_LIST BASELINE_BOARDS AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}")
        # If CMakeLists.txt is missing but an .ioc file is present, scaffold it automatically
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/CMakeLists.txt")
            file(GLOB BOARD_IOCS "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/*.ioc")
            if(BOARD_IOCS)
                find_package(Python3 COMPONENTS Interpreter QUIET)
                if(Python3_FOUND)
                    execute_process(
                        COMMAND "${Python3_EXECUTABLE}" -c "import sys; from pathlib import Path; sys.path.insert(0, str(Path(r'${CMAKE_CURRENT_SOURCE_DIR}').parent)); from tools.rov_cli.build import scaffold_board_cmakelists; scaffold_board_cmakelists(Path(r'${CMAKE_CURRENT_SOURCE_DIR}/${DIR}'))"
                        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
                        OUTPUT_QUIET ERROR_QUIET
                    )
                endif()
            endif()
        endif()

        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/CMakeLists.txt")
            add_subdirectory(${DIR})
        endif()
    endif()
endforeach()
