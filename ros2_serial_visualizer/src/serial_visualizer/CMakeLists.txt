cmake_minimum_required(VERSION 3.8)
project(serial_visualizer)

find_package(ament_cmake_auto REQUIRED)

#xmlから依存を見つける
ament_auto_find_build_dependencies()

# 実行ファイルの生成
ament_auto_add_executable(
  dammy_feedback_node src/dammy_feedback_node.cpp
)

# 実行ファイルの生成
ament_auto_add_executable(
  serial_convert_node src/serial_convert_node.cpp
)
#コンポーネントはこちら
#ament_auto_add_library(example src/example.cpp)
#rclcpp_components_register_nodes(example "name_space::SampleClass")

#shareフォルダにインストールしておきたいフォルダを指定する
#install(DIRECTORY
#  config
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# msg/srv generation
file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
#file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
#file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_msg_files}  # msg使わないならコメントアウト
        #${${PROJECT_NAME}_srv_files}  # srv使わないならコメントアウト
        #${${PROJECT_NAME}_action_files}  # action使わないならコメントアウト
        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dammy_feedback_node "${cpp_typesupport_target}")

#ライブラリのビルド
ament_auto_package()
