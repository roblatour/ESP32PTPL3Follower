idf_component_register(
    SRCS
        "main.cpp"
        "clock_owner.cpp"
        "ethernet.cpp"
        "follower_core.cpp"
        "app_metadata.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_eth
        esp_event
        esp_timer
        esp_netif
        lwip
        esp_driver_gpio
        esp_hw_support
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

if(DEFINED FOLLOWER_PTP_IPV6)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FOLLOWER_PTP_IPV6=${FOLLOWER_PTP_IPV6})
endif()

# Metadata definitions injected into the main component
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    APP_PROJECT_NAME="${CMAKE_PROJECT_NAME}"
    APP_VERSION="${PROJECT_VER}"
    APP_COPYRIGHT="Copyright Rob Latour, 2026"
    APP_LICENSE="MIT"
    APP_HOMEPAGE="https://github.com/roblatour/ESP32PTPL3Follower"
    APP_BUILD_DATE=__DATE__
    APP_BUILD_TIME=__TIME__
    APP_GIT_SHA="unavailable"
) 
