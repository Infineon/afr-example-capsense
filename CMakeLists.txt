#
# Application specific CMakeLists.txt file
#   Included from vendors/cypress/psoc6/cmake/cy_kit_utils.cmake
#
# Add app specific sources, defines, includes here
#
#--------------------------------------------------------------------
# This app dir
get_filename_component(CY_APP_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

#--------------------------------------------------------------------
# Additional defines
target_compile_definitions(${AFR_TARGET_APP_NAME} PUBLIC
    "-DCY_RETARGET_IO_CONVERT_LF_TO_CRLF"
    )
    
# Application sources
# ( main.c already included in vendors/cypress/psoc6/cmake/cy_defines.cmake)
# All the source files in this example directory are recursively added through
# cy_kit_generate() function in vendors/cypress/psoc6/cmake/cy_kit_utils.cmake.
# target_sources(${AFR_TARGET_APP_NAME} PUBLIC "" )
target_sources(${AFR_TARGET_APP_NAME} PUBLIC 
    "${AFR_DEMOS_DIR}/demo_runner/aws_demo.c"
    "${AFR_DEMOS_DIR}/demo_runner/iot_demo_freertos.c"
    "${AFR_DEMOS_DIR}/dev_mode_key_provisioning/src/aws_dev_mode_key_provisioning.c"
    "${AFR_DEMOS_DIR}/network_manager/aws_iot_network_manager.c"
)

# Additional include dirs
# target_include_directories(${AFR_TARGET_APP_NAME} PUBLIC "" )
target_include_directories(${AFR_TARGET_APP_NAME} PUBLIC 
    "${CMAKE_CURRENT_LIST_DIR}/source"
    "${CMAKE_CURRENT_LIST_DIR}/source/cJSON"
    "${AFR_DEMOS_DIR}/dev_mode_key_provisioning/include"
    "${AFR_DEMOS_DIR}/network_manager"
    "${AFR_MODULES_DIR}/3rdparty/pkcs11"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/pkcs11/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/include/platform"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/include"
)

target_link_libraries(${AFR_TARGET_APP_NAME} PUBLIC
    "AFR::mqtt"
)  

# AWS Configuration directory. Set if CE is not owning config_files (i.e when  
# config_files directory is not present inside the CE directory). 
# When the path is empty, config_files directory inside the CE directory will be
# used.
#set(CY_APP_CONFIG_DIR "${cy_board_dir}/aws_demos/config_files")

# AWS IOT Configuration directory. Set if CE is not owning include directory. 
# (i.e when include directory is not present inside the CE directory.). When the
# path is empty, include directory inside the CE directory will be used. 
set(CY_APP_IOT_CONFIG_DIR "${AFR_DEMOS_DIR}/include")

# Additional libs or link options we need
# target_link_options(${AFR_TARGET_APP_NAME} PUBLIC "<lib_name>" )

# Extra linker options
# target_link_options(${AFR_TARGET_APP_NAME} PUBLIC "" )

#--------------------------------------------------------------------
# Disable AFR aws_demos Modules
# These AFR aws_demos modules are not needed for the capsense to build
set(CY_APP_DISABLE_AFR_MODULES
    "defender"
    "greengrass"
    "shadow"
    )

