cmake_minimum_required(VERSION 3.16)
project(orb_slam3_fw LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(fw REQUIRED)
# essential dependencies 
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(tf2_msgs REQUIRED)
find_package(mcap_vendor REQUIRED)  # euroc_mcap_driver reads .mcap files
find_package(Eigen3 REQUIRED)

# Builds every block under drivers/, algorithms/, types/, params/ and filesystem/plugins/ and writes the package's manifest.
# Blocks using ROS 2 message types: fw_add_blocks(LIBRARIES fw::ros ${sensor_msgs_TARGETS}) after
# find_package(sensor_msgs REQUIRED). _TARGETS brings the headers too; _LIBRARIES does not.
fw_add_blocks(LIBRARIES
  fw::ros
  fw::image
  Eigen3::Eigen
  ${orb_msgs_target}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_msgs_TARGETS}
  mcap_vendor::mcap
  opencv_imgcodecs
)

# fw_add_blocks globs every .cpp under algorithms/, hidden folders too: leave those out (e.g. a kept
# reference implementation such as algorithms/orb_matcher/.reference, whose tests have their own main()).
get_target_property(FW_BLOCK_SOURCES ${FW_BLOCKS_TARGET} SOURCES)
list(FILTER FW_BLOCK_SOURCES EXCLUDE REGEX "/\\.[^/]+/")
set_property(TARGET ${FW_BLOCKS_TARGET} PROPERTY SOURCES ${FW_BLOCK_SOURCES})

# orb_matcher's hot loops (Hamming, sub-pixel sampling) are ~40x slower unoptimized; a plain
# `colcon build` sets no CMAKE_BUILD_TYPE, so optimize its sources in that case.
if(NOT CMAKE_BUILD_TYPE)
  file(GLOB_RECURSE ORB_MATCHER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/orb_matcher/*.cpp)
  list(FILTER ORB_MATCHER_SOURCES EXCLUDE REGEX "/\\.[^/]+/")
  set_source_files_properties(${ORB_MATCHER_SOURCES} PROPERTIES COMPILE_OPTIONS "-O2")
endif()

# orb_matcher library (algorithms/orb_matcher without the block) for its tests (not installed):
#   build/orb_slam3_fw/orb_matcher_kb8_test        KB8 model, rig, calib::StereoImuRig -> rig (radtan fit)
#   build/orb_slam3_fw/orb_matcher_epipolar_test   matcher on synthetic scenes vs ORB-SLAM3 fisheye [--trials N]
#   build/orb_slam3_fw/orb_matcher_subpixel_test   sub-pixel refinement on rendered planes [--trials N]
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
add_library(orb_matcher_core STATIC
  types/camera/kannala_brandt.cpp types/camera/stereo_rig.cpp types/camera/rig_from_calib.cpp
  algorithms/orb_matcher/stereo_matcher.cpp
  algorithms/orb_matcher/core/bearing.cpp algorithms/orb_matcher/core/subpixel.cpp)
target_include_directories(orb_matcher_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orb_matcher_core PUBLIC Eigen3::Eigen opencv_core fw::core)
if(NOT CMAKE_BUILD_TYPE)
  target_compile_options(orb_matcher_core PUBLIC -O2)   # the tests measure time
endif()
foreach(t kb8 epipolar subpixel)
  set(src ${t}_test)
  if(t STREQUAL "epipolar" OR t STREQUAL "subpixel")
    set(src ${t}_synth_test)
  endif()
  add_executable(orb_matcher_${t}_test tests/orb_matcher/${src}.cpp)
  target_link_libraries(orb_matcher_${t}_test orb_matcher_core opencv_imgproc)
endforeach()
if(BUILD_TESTING)
  add_test(NAME orb_matcher_kb8_test COMMAND orb_matcher_kb8_test)
  add_test(NAME orb_matcher_epipolar_test COMMAND orb_matcher_epipolar_test --trials 50)
  add_test(NAME orb_matcher_subpixel_test COMMAND orb_matcher_subpixel_test --trials 10)
endif()

# types/ tests (not installed): build/orb_slam3_fw/types_<name>_test
#   so3             types/geometry/so3.hpp: exp/log vs AngleAxis, Jacobians vs numerical, ORB-SLAM3 formulas
#   preintegration  types/imu/preintegrated + imu_preintegration's interval vs tests/fixtures/orb_slam3/preintegration
set(types_so3_test_SOURCES)
set(types_preintegration_test_SOURCES types/imu/preintegrated.cpp)
foreach(t so3 preintegration)
  add_executable(types_${t}_test tests/types/${t}_test.cpp ${types_${t}_test_SOURCES})
  target_include_directories(types_${t}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(types_${t}_test PRIVATE ORB_SLAM3_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/orb_slam3")
  target_link_libraries(types_${t}_test Eigen3::Eigen fw::core)
  if(BUILD_TESTING)
    add_test(NAME types_${t}_test COMMAND types_${t}_test)
  endif()
endforeach()

# orb_extractor: no fused multiply-add, so the Harris responses and BRIEF rotations round exactly as
# the GPU version's `precise` arithmetic (same keypoints and descriptors).
set_source_files_properties(algorithms/orb_extractor/orb_cpu.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")

# orb_extractor GPU backend (Vulkan compute, algorithms/orb_extractor/vk): built when Vulkan and
# glslangValidator are found (the dev container has both), else the block runs on the CPU only.
# Each shader is compiled to a header (gen/spv_<name>.h) embedding the SPIR-V words; SPIR-V is GPU
# independent, so a host glslangValidator also serves a cross build.
find_package(Vulkan QUIET)
find_program(GLSLANG glslangValidator)
option(ORB_EXTRACTOR_VULKAN "Build the orb_extractor GPU (Vulkan) backend when Vulkan is found" ON)
if(ORB_EXTRACTOR_VULKAN AND Vulkan_FOUND AND GLSLANG)
  set(ORB_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/orb_extractor/vk/shaders)
  set(ORB_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/orb_extractor_gen)
  file(MAKE_DIRECTORY ${ORB_GEN_DIR})
  set(ORB_SHADERS blur cell_list describe fast fast_cells fast_full fast_mark grid_rank grid_scan grid_scatter
                  grid_top harris nms post_nms rank_thresh reset resize resize_any select select_tie thresh)
  set(ORB_SPV_HEADERS)
  foreach(s ${ORB_SHADERS})
    set(out ${ORB_GEN_DIR}/spv_${s}.h)
    add_custom_command(
      OUTPUT ${out}
      COMMAND ${GLSLANG} -V --target-env vulkan1.1 -I${ORB_SHADER_DIR} --vn spv_${s} -o ${out} ${ORB_SHADER_DIR}/${s}.comp
      DEPENDS ${ORB_SHADER_DIR}/${s}.comp ${ORB_SHADER_DIR}/common.glsl
      COMMENT "SPIR-V ${s}.comp")
    list(APPEND ORB_SPV_HEADERS ${out})
  endforeach()
  # orb_matcher's sub-pixel refinement on the GPU (algorithms/orb_matcher/gpu), same device as the extractor
  set(ORB_MATCHER_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/orb_matcher/gpu/shaders)
  foreach(s subpixel)
    set(out ${ORB_GEN_DIR}/spv_${s}.h)
    add_custom_command(
      OUTPUT ${out}
      COMMAND ${GLSLANG} -V --target-env vulkan1.1 --vn spv_${s} -o ${out} ${ORB_MATCHER_SHADER_DIR}/${s}.comp
      DEPENDS ${ORB_MATCHER_SHADER_DIR}/${s}.comp
      COMMENT "SPIR-V ${s}.comp")
    list(APPEND ORB_SPV_HEADERS ${out})
  endforeach()
  add_custom_target(orb_extractor_shaders DEPENDS ${ORB_SPV_HEADERS})
  # Vulkan create-info structs are written {VK_STRUCTURE_TYPE_...}: the other members zero on purpose
  set_source_files_properties(algorithms/orb_extractor/vk/orb_vk.cpp algorithms/orb_extractor/vk/orb_vk_slam.cpp
                              types/gpu/context.cpp algorithms/orb_matcher/gpu/subpixel_vk.cpp
                              PROPERTIES COMPILE_OPTIONS "-Wno-missing-field-initializers")
  add_dependencies(${FW_BLOCKS_TARGET} orb_extractor_shaders)
  target_include_directories(${FW_BLOCKS_TARGET} PRIVATE ${ORB_GEN_DIR})
  target_compile_definitions(${FW_BLOCKS_TARGET} PRIVATE ORB_HAVE_VULKAN=1)
  target_link_libraries(${FW_BLOCKS_TARGET} PRIVATE Vulkan::Vulkan)
  message(STATUS "orb_extractor: Vulkan backend on (${Vulkan_LIBRARY}, ${GLSLANG})")

  # GPU vs CPU backend check, bit for bit (not installed): build/orb_slam3_fw/orb_gpu_vs_cpu [image.pgm ...]
  add_executable(orb_gpu_vs_cpu tests/orb_gpu_vs_cpu.cpp algorithms/orb_extractor/orb_cpu.cpp
                                algorithms/orb_extractor/vk/orb_vk.cpp algorithms/orb_extractor/vk/orb_vk_slam.cpp
                                types/gpu/context.cpp)
  add_dependencies(orb_gpu_vs_cpu orb_extractor_shaders)
  target_include_directories(orb_gpu_vs_cpu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${ORB_GEN_DIR} ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(orb_gpu_vs_cpu PRIVATE ORB_HAVE_VULKAN=1)
  target_link_libraries(orb_gpu_vs_cpu Vulkan::Vulkan ${OpenCV_LIBS})

  # orb_matcher sub-pixel refinement, GPU (shader) vs CPU (SubpixelRefiner) on the same searches, and the
  # extractor's GPU pyramid handover (not installed): build/orb_slam3_fw/orb_matcher_subpixel_gpu_test
  add_executable(orb_matcher_subpixel_gpu_test tests/orb_matcher/subpixel_gpu_test.cpp algorithms/orb_extractor/orb_cpu.cpp
                 algorithms/orb_matcher/gpu/subpixel_vk.cpp types/gpu/context.cpp
                 algorithms/orb_extractor/vk/orb_vk.cpp algorithms/orb_extractor/vk/orb_vk_slam.cpp)
  add_dependencies(orb_matcher_subpixel_gpu_test orb_extractor_shaders)
  target_include_directories(orb_matcher_subpixel_gpu_test PRIVATE ${ORB_GEN_DIR})
  target_compile_definitions(orb_matcher_subpixel_gpu_test PRIVATE ORB_HAVE_VULKAN=1)
  target_link_libraries(orb_matcher_subpixel_gpu_test orb_matcher_core Vulkan::Vulkan opencv_imgproc)
  if(BUILD_TESTING)
    add_test(NAME orb_matcher_subpixel_gpu_test COMMAND orb_matcher_subpixel_gpu_test)
  endif()
else()
  message(STATUS "orb_extractor: Vulkan or glslangValidator not found, CPU backend only")
endif()

ament_package()
