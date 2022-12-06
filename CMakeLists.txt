project(ISO_object LANGUAGES C CXX)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

# Swig flags
set(WITH_SWIG OFF CACHE BOOL "Enable swigging")
set(SWIG_WITH_JAVA OFF CACHE BOOL "Swig to target-language java")
set(SWIG_WITH_PYTHON OFF CACHE BOOL "Swig to target-language python")
if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
endif()

# Manage namespace of the project
set(ISOOBJECT_LIBRARY ${PARENT_TARGET_NAMESPACE}${PROJECT_NAME})
set(ISOOBJECT_TEST ${ISOOBJECT_LIBRARY}Demo)

#ISO22133 library/headers
add_subdirectory(../iso22133 ${CMAKE_CURRENT_BINARY_DIR}/iso22133)
set(ISO22133_LIBRARY ISO22133)
find_path(ISO22133_HEADER_DIR
    NAMES "iso22133.h"
    NO_DEFAULT_PATH
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../iso22133"
    REQUIRED
)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS system)

# Library target
add_library(${ISOOBJECT_LIBRARY} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/trajDecoder.cpp
)

target_include_directories(${ISOOBJECT_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
    ${ISO22133_HEADER_DIR}
)

target_link_libraries(${ISOOBJECT_LIBRARY}
    ${ISO22133_LIBRARY}
    atomic
    pthread
)

set_property(TARGET ${ISOOBJECT_LIBRARY} PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133object.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133state.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/trajDecoder.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot/signal.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/tcpServer.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/udpServer.hpp
)

# Demo target
add_executable(${ISOOBJECT_TEST} demoIsoObject.cpp)

target_include_directories(${ISOOBJECT_TEST} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${ISOOBJECT_TEST} LINK_PUBLIC
    ${ISOOBJECT_LIBRARY}
)

install(CODE "MESSAGE(STATUS \"Installing target ${ISOOBJECT_LIBRARY}\")")
install(TARGETS ${ISOOBJECT_LIBRARY}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
message(("BOOST:" ${Boost_INCLUDE_DIRS}))

# SWIG
if (WITH_SWIG)
    find_package(SWIG REQUIRED)
    if(NOT SWIG_FOUND)
        message(WARNING "SWIG not found")
    else()
        #Policy for swig_add_library to put target inside first argument to the function
        cmake_policy(SET CMP0078 NEW)
        #Policy for passing SWIG_MODULE_NAME to swig compiler as -module <name>
        cmake_policy(SET CMP0086 NEW)

        include(${SWIG_USE_FILE})

        if(SWIG_WITH_JAVA)
            if (NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Android")
                find_package(JNI REQUIRED)
            endif()
            SET(CMAKE_SWIG_FLAGS -package org.asta.isoObject)
        elseif(SWIG_WITH_PYTHON)
            find_package(Python COMPONENTS Development)
            SET(CMAKE_SWIG_FLAGS "")
        endif()

        set(ISO22133_SWIGGED isoObject_wrap)

        set_property(SOURCE isoObject.i PROPERTY CPLUSPLUS ON)
        set_property(SOURCE isoObject.i PROPERTY INCLUDE_DIRECTORIES
            ${CMAKE_CURRENT_SOURCE_DIR}/inc
            ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
            ${ISO22133_HEADER_DIR}
            ${Python_INCLUDE_DIRS} #Note that cmake ignored empty variables
            ${JNI_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
        )
        swig_add_library(${ISO22133_SWIGGED}
            TYPE SHARED
            LANGUAGE ${SWIG_TARGET_LANG}
            SOURCES isoObject.i
        )
        set_property(TARGET ${ISO22133_SWIGGED} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
        target_include_directories(${ISO22133_SWIGGED} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/inc
            ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
            ${ISO22133_HEADER_DIR}
            ${Python_INCLUDE_DIRS}
            ${JNI_INCLUDE_DIRS}
        )

        target_link_libraries(${ISO22133_SWIGGED}
            ${ISOOBJECT_LIBRARY}
            ${ISO22133_LIBRARY}
            atomic
            ${Python_LIBRARIES}
            ${JNI_LIBRARIES}
        )
    endif()
endif()