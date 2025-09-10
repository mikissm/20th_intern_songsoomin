cmake_minimum_required(VERSION 3.19)
project(project LANGUAGES CXX)

# Qt6 6.5 이상, Widgets + Network 사용
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Network)

qt_standard_project_setup()

# UI 경로 지정
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/ui)

# 소스/헤더/UI 경로 설정
set(PROJECT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/weapon.cpp
    src/connectiondialog.cpp
    include/mainwindow.h
    include/weapon.h
    include/connectiondialog.h
    ui/mainwindow.ui
)

qt_add_executable(project
    WIN32 MACOSX_BUNDLE
    ${PROJECT_SOURCES}
)

# include 디렉터리 추가
target_include_directories(project PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(project
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Network
)

# 설치 관련 
include(GNUInstallDirs)

install(TARGETS project
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET project
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})

