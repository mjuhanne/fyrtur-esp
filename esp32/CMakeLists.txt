# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ../../components;../main;../fyrtur-console)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(fyrtur-esp32)

include(../../components/node-framework/iot.cmake)

add_custom_target(upload-stm32-bin
	COMMAND sftp $ENV{IOT_FIRMWARE_SERVER_WWW_ROOT_URI} <<< $$'put ../../fyrtur-stm32.bin'
	COMMAND echo `md5 -q ../../fyrtur-stm32.bin` fyrtur-stm32.bin > ../../fyrtur-stm32.bin.md5
	COMMAND sftp $ENV{IOT_FIRMWARE_SERVER_WWW_ROOT_URI} <<< $$'put ../../fyrtur-stm32.bin.md5'
	)	

add_custom_target(mqtt-stm32-ota DEPENDS upload-stm32-bin
	COMMAND mqtt pub -t /home/control/${PROJECT_NAME}/stm32update -m '$ENV{IOT_FIRMWARE_SERVER_URI}/fyrtur-stm32.bin FETCH'
    )


