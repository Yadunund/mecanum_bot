cmake_minimum_required(VERSION 3.5)
project(mecanum_bot_bringup)

find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  ##############################################################################
  # Generate Gz world and download Models
  ##############################################################################

  # generate Ignition simulation world
  set(ign_output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name}_ign)
  set(ign_output_world_path ${ign_output_dir}/${output_world_name}.world)
  set(ign_output_model_dir ${ign_output_dir}/models)

  add_custom_command(
    OUTPUT ${ign_output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator ignition ${map_path} ${ign_output_world_path} ${ign_output_model_dir}
    DEPENDS ${map_path}
  )

  install(
    DIRECTORY ${ign_output_dir} ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()

install(
  DIRECTORY config launch map rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
