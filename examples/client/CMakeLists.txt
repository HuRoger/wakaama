cmake_minimum_required (VERSION 3.0)

project (lwm2mclient)

option(DTLS "Enable DTLS" OFF)

include(${CMAKE_CURRENT_LIST_DIR}/../../core/wakaama.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../shared/shared.cmake)

add_definitions(-DLWM2M_CLIENT_MODE -DLWM2M_BOOTSTRAP -DLWM2M_SUPPORT_JSON -DLWM2M_BOOTSTRAP)
add_definitions(${SHARED_DEFINITIONS} ${WAKAAMA_DEFINITIONS})

include_directories (${WAKAAMA_SOURCES_DIR} ${SHARED_INCLUDE_DIRS})

SET(SOURCES
    lwm2mclient.c
    lwm2mclient.h
    BBBIOlib/BBBio_lib/BBBiolib.c
    BBBIOlib/BBBio_lib/BBBiolib.h
    BBBIOlib/BBBio_lib/BBBiolib_ADCTSC.c
    BBBIOlib/BBBio_lib/BBBiolib_ADCTSC.h
    BBBIOlib/BBBio_lib/BBBiolib_McSPI.c
    BBBIOlib/BBBio_lib/BBBiolib_McSPI.h
    BBBIOlib/BBBio_lib/BBBiolib_PWMSS.c
    BBBIOlib/BBBio_lib/BBBiolib_PWMSS.h
    system_api.c
    object_security.c
    object_server.c
    object_device.c
    object_firmware.c
    object_location.c
    object_connectivity_moni.c
    object_connectivity_stat.c
    object_access_control.c
    test_object.c
    object_light.c
    object_distance.c
    )

add_executable(${PROJECT_NAME} ${SOURCES} ${WAKAAMA_SOURCES} ${SHARED_SOURCES})

FIND_PACKAGE(Threads)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(lwm2mclient
    general -lprussdrv
    )

# Add WITH_LOGS to debug variant
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:WITH_LOGS>)
