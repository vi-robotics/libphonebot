# Project Definition.
cmake_minimum_required(VERSION 3.8)
project(libphonebot-swig)

# Find SWIG.
find_package(SWIG REQUIRED)
include(UseSWIG)

# Find Java/JNI.
find_package(Java COMPONENTS Development REQUIRED)
if(NOT ANDROID)
    #jni is available by default on android
    find_package(JNI REQUIRED)
    include_directories(${JNI_INCLUDE_DIRS})
endif()

# Set couple of package parameters.
set(PACKAGE_NAME phonebot.control)
set(UseSWIG_TARGET_NAME_PREFERENCE LEGACY)
set(LIBRARY_NAME phonebotjava)

# TODO(yycho0108): multiple sources support
set_property(SOURCE Phonebot.i PROPERTY CPLUSPLUS ON)

list(APPEND CMAKE_SWIG_FLAGS -package ${PACKAGE_NAME})

#set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/src/main/jniLibs)
#set(SWIG_OUTFILE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/src/main/jniLibs)
#set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/src/main/java/_gen)

swig_add_library(${LIBRARY_NAME}
    TYPE SHARED
    LANGUAGE java
    SOURCES Phonebot.i)
#add_jar(testphonebotjar
#    SOURCES FILE(GLOB ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/src/main/java/_gen/*.java)
#    )

swig_link_libraries(${LIBRARY_NAME}
    phonebot)

target_include_directories(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME}
    PRIVATE ${JNI_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/..)

set_target_properties(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME} PROPERTIES
    INSTALL_RPATH   "$ORIGIN")

# .java -> .jar
FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classes")
ADD_CUSTOM_COMMAND(TARGET ${LIBRARY_NAME} POST_BUILD
    COMMAND "${Java_JAVAC_EXECUTABLE}" -d classes *.java
    COMMAND "${Java_JAR_EXECUTABLE}" -cfM ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/libs/${LIBRARY_NAME}.jar -C classes .
    )
