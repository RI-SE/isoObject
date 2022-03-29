project(ISO_object LANGUAGES C CXX)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_BUILD_TYPE Debug)

# Manage namespace of the project
set(ISOOBJECT_LIBRARY ${PARENT_TARGET_NAMESPACE}${PROJECT_NAME})
set(ISOOBJECT_TEST ${ISOOBJECT_LIBRARY}Test)

find_library(ISO22133_LIBRARY ISO22133)
if (NOT ISO22133_LIBRARY)
        message("ISO22133 library not found - attempting search in local build tree")
        add_subdirectory(../iso22133 ${CMAKE_CURRENT_BINARY_DIR}/iso22133)
        set(ISO22133_LIBRARY ISO22133)
        message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
        find_path(ISO22133_HEADER_DIR
            NAMES "iso22133.h"
            NO_CMAKE_FIND_ROOT_PATH
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../iso22133"
            REQUIRED)
else()
        find_path(ISO22133_HEADER_DIR iso22133.h REQUIRED)
endif()
message("ISO22133_HEADER_DIR: ${ISO22133_HEADER_DIR}")

find_library(SOCKET_LIBRARY TCPUDPSocket)
if (NOT SOCKET_LIBRARY)
        message("Socket library not found - attempting search in local build tree")
        add_subdirectory(../sockets ${CMAKE_CURRENT_BINARY_DIR}/sockets)
        set(SOCKET_LIBRARY TCPUDPSocket)
        find_path(SOCKET_HEADER_DIR
            NAMES "socket.hpp"
            NO_CMAKE_FIND_ROOT_PATH
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../sockets"
            REQUIRED)
else()
        find_path(SOCKET_HEADER_DIR socket.hpp REQUIRED)
endif()
message("SOCKET_HEADER_DIR: ${SOCKET_HEADER_DIR}")

include(GNUInstallDirs)

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
        ${SOCKET_LIBRARY}
        ${ISO22133_LIBRARY}
        atomic
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
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/testTestTest.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot/signal.hpp
)

# Test target
add_executable(${ISOOBJECT_TEST} testIsoObject.cpp)

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

# SWIG
find_package(SWIG COMPONENTS java)
if(NOT SWIG_FOUND)
        message(WARNING "SWIG not found")
else()
        include(${SWIG_USE_FILE})
        find_package(JNI REQUIRED)

        set(ISO22133_JAVA isoObject_wrap)
        set_property(SOURCE isoObject.i PROPERTY CPLUSPLUS ON)
        set_property(SOURCE isoObject.i PROPERTY INCLUDE_DIRECTORIES
                ${CMAKE_CURRENT_SOURCE_DIR}/inc
                ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
                ${SOCKET_HEADER_DIR}
                ${ISO22133_HEADER_DIR}
                ${JNI_INCLUDE_DIRS}
        )
        swig_add_library(${ISO22133_JAVA}
                TYPE SHARED
                LANGUAGE java
                SOURCES isoObject.i
        )
        set_property(TARGET ${ISO22133_JAVA} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
        target_include_directories(${ISO22133_JAVA} PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/inc
                ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
                ${SOCKET_HEADER_DIR}
                ${ISO22133_HEADER_DIR}
                ${JNI_INCLUDE_DIRS}
        )

        target_link_libraries(${ISO22133_JAVA}
                ${ISOOBJECT_LIBRARY}
                ${SOCKET_LIBRARY}
                ${ISO22133_LIBRARY}
                ${JNI_LIBRARIES}
                atomic
        )
endif()
