cmake_minimum_required(VERSION 3.5)

project(hw3_package VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# UI 경로 추가
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/ui)

set(PROJECT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.h
    ui/mainwindow.ui
    # resources/resources.qrc   # 리소스 쓸 경우 추가
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(hw3_package
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(hw3_package SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(hw3_package
            ${PROJECT_SOURCES}
        )
    endif()
endif()

# include 폴더 추가
target_include_directories(hw3_package PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(hw3_package PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(hw3_package PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(hw3_package)
endif()

