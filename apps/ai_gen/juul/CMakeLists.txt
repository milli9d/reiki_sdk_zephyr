# Copyright (c) 2026 Milind Singh
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

set(REIKI_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../)

list(APPEND EXTRA_ZEPHYR_MODULES
    ${REIKI_ROOT_DIR}/sdk/
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(juul)

target_sources(app PRIVATE src/main.c)
