MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")

    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()

    SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(subdirs ${CMAKE_CURRENT_SOURCE_DIR})

foreach(subdir ${subdirs})
    message(STATUS "Adding subdirectory: ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}")
    add_subdirectory(${subdir})
endforeach()
