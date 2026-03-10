# Copyright (c) 2025 Arduino SA
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_ESP_HOSTED_SDIO)

zephyr_library_named(esp_hosted_sdio)

set(NANOPB_DIR ${ZEPHYR_BASE}/../modules/lib/nanopb)
set(NANOPB_GENERATOR ${NANOPB_DIR}/generator/nanopb_generator.py)

set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_FILE ${PROTO_DIR}/esp_hosted_rpc.proto)
set(GEN_C_FILE ${PROTO_DIR}/esp_hosted_rpc.pb.c)
set(GEN_H_FILE ${PROTO_DIR}/esp_hosted_rpc.pb.h)

add_custom_command(
    OUTPUT ${GEN_C_FILE} ${GEN_H_FILE}
    COMMAND ${PYTHON_EXECUTABLE} ${NANOPB_GENERATOR}
            -I${PROTO_DIR}
            --output-dir=${PROTO_DIR}
            ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating nanopb sources for esp_hosted_sdio"
)

zephyr_library_sources(
  esp_hosted_sdio_hal.c
  esp_hosted_sdio_cmd.c
  esp_hosted_sdio_wifi.c
  ${GEN_C_FILE}
)

zephyr_include_directories(
  ./
  ${PROTO_DIR}
)

endif()
