# -*- Mode: CMake; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*- */

MACRO(LEGIT_GENERATE_HEADERS DST)

    SET(TRACKERS_FILE "${DST}/trackers.h")

    FILE(WRITE "${TRACKERS_FILE}" "// Autogenerated file, do not edit! \n")

    FOREACH(TRACKER_HEADER ${LEGIT_TRACKERS})

        GET_SOURCE_FILE_PROPERTY(TRACKER_CLASS ${TRACKER_HEADER} LEGIT_TRACKER)

       # MESSAGE("Register: ${TRACKER_CLASS}")

        FILE(APPEND "${TRACKERS_FILE}" "#include \"${TRACKER_HEADER}\"\n")

    ENDFOREACH(TRACKER_HEADER)


ENDMACRO(LEGIT_GENERATE_HEADERS)


MACRO (LEGIT_ADD_SOURCES)
    FILE (RELATIVE_PATH _relPath "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")

    FOREACH (_src ${ARGN})
        IF (_relPath)
            LIST (APPEND LEGIT_SOURCES "${_relPath}/${_src}")
        ELSE (_relPath)
            LIST (APPEND LEGIT_SOURCES "${_src}")
        ENDIF (_relPath)
    ENDFOREACH()
    IF (_relPath)
        SET (LEGIT_SOURCES ${LEGIT_SOURCES} PARENT_SCOPE)
    ENDIF(_relPath)
ENDMACRO (LEGIT_ADD_SOURCES)

MACRO (LEGIT_REGISTER_TRACKER HEADER TRACKER LOCATION)
    FILE (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}/libs/legit/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    IF (_relPath)
        LIST (APPEND LEGIT_TRACKERS "${_relPath}/${HEADER}")
        SET_SOURCE_FILES_PROPERTIES("${_relPath}/${HEADER}" PROPERTIES LEGIT_TRACKER $TRACKER)
    ELSE (_relPath)
        LIST (APPEND LEGIT_TRACKERS "${HEADER}")
        SET_SOURCE_FILES_PROPERTIES("${HEADER}" PROPERTIES LEGIT_TRACKER $TRACKER)
    ENDIF (_relPath)
    IF (_relPath)
        SET (LEGIT_TRACKERS ${LEGIT_TRACKERS} PARENT_SCOPE)
    ENDIF(_relPath)
ENDMACRO (LEGIT_REGISTER_TRACKER)

MACRO (LEGIT_REGISTER_CONFIGURATION CONFIG)
    FILE (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}/libs/legit" "${CMAKE_CURRENT_SOURCE_DIR}")
    LIST (APPEND LEGIT_CONFIGURATION "${CMAKE_CURRENT_SOURCE_DIR}/${HEADER}")
    IF (_relPath)
        SET (LEGIT_CONFIGURATION ${LEGIT_CONFIGURATION} PARENT_SCOPE)
    ENDIF(_relPath)
ENDMACRO (LEGIT_REGISTER_CONFIGURATION)


MACRO (LEGIT_PROPAGATE)
SET (LEGIT_CONFIGURATION ${LEGIT_CONFIGURATION} PARENT_SCOPE)
SET (LEGIT_TRACKERS ${LEGIT_TRACKERS} PARENT_SCOPE)
SET (LEGIT_SOURCES ${LEGIT_SOURCES} PARENT_SCOPE)
ENDMACRO (LEGIT_PROPAGATE)
