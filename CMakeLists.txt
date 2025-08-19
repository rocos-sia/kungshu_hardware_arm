cmake_minimum_required(VERSION 3.8)
project(kungshu_hardware_arm)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(kungshu_msgs REQUIRED)

# add soem cmake path
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/soem-2.0.0-Linux-x86_64/cmake")


find_package(soem REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)


add_executable(arm_node src/main.cpp src/fieldbus.cpp src/arm_node.cpp)
target_include_directories(arm_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(arm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(arm_node
        fmt::fmt
        spdlog::spdlog
        soem
)

ament_target_dependencies(arm_node
        rclcpp
        kungshu_msgs
)

install(TARGETS arm_node
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
