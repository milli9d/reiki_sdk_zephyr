#!/usr/bin/env bash
#
# Copyright (c) 2026 Milind Singh
# SPDX-License-Identifier: MIT
#
# Build, flash and monitor any app in this repo from one place.
#
#   ./zephyr.sh                      interactive menu
#   ./zephyr.sh <command> [options]  run one command and exit
#
# Run ./zephyr.sh help for the commands and options. The selected app, board
# and serial port are remembered between runs in .zephyr_cache.
#
# Works with bash 3.2 (macOS) and later; no external menu tools needed.

set -u

# ============================================================================ #
# Paths and defaults
# ============================================================================ #

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="${REPO_DIR}/apps"
BOARDS_DIR="${REPO_DIR}/sdk/boards"
BUILD_ROOT="${REPO_DIR}/build"
STATE_FILE="${REPO_DIR}/.zephyr_cache"
SETUP_SCRIPT="${REPO_DIR}/scripts/build/env/setup.sh"

# West workspace holding zephyr/ and the Python venv; override with the env var
ZEPHYR_WORKSPACE="${ZEPHYR_WORKSPACE:-${HOME}/zephyrproject}"

if [[ -t 1 ]]; then
    C_DIM=$'\033[0;90m'
    C_RED=$'\033[0;91m'
    C_GREEN=$'\033[0;92m'
    C_YELLOW=$'\033[0;93m'
    C_BLUE=$'\033[0;94m'
    C_CYAN=$'\033[0;96m'
    C_BOLD=$'\033[1m'
    C_RESET=$'\033[0m'
else
    C_DIM="" C_RED="" C_GREEN="" C_YELLOW="" C_BLUE="" C_CYAN="" C_BOLD="" C_RESET=""
fi

APP=""
BOARD=""
PORT=""

# ============================================================================ #
# Output helpers
# ============================================================================ #

info() { echo "${C_CYAN}==>${C_RESET} $*"; }
ok() { echo "${C_GREEN}✔${C_RESET} $*"; }
warn() { echo "${C_YELLOW}!${C_RESET} $*" >&2; }
err() { echo "${C_RED}✘${C_RESET} $*" >&2; }

# Print a key/value row of the status header
row() { printf "  ${C_DIM}%-8s${C_RESET} %s\n" "$1" "$2"; }

# ============================================================================ #
# State
# ============================================================================ #

load_state() {
    [[ -f "${STATE_FILE}" ]] || return 0
    local key value
    while IFS='=' read -r key value; do
        case "${key}" in
            app) APP="${value}" ;;
            board) BOARD="${value}" ;;
            port) PORT="${value}" ;;
        esac
    done <"${STATE_FILE}"
}

save_state() {
    printf 'app=%s\nboard=%s\nport=%s\n' "${APP}" "${BOARD}" "${PORT}" >"${STATE_FILE}"
}

# ============================================================================ #
# Discovery
# ============================================================================ #

# Apps are directories under apps/ with both CMakeLists.txt and prj.conf,
# printed relative to apps/ (e.g. "shell", "ai_gen/juul")
list_apps() {
    find "${APPS_DIR}" -name prj.conf -not -path '*/build/*' 2>/dev/null |
        while read -r conf; do
            local dir
            dir="$(dirname "${conf}")"
            [[ -f "${dir}/CMakeLists.txt" ]] && echo "${dir#"${APPS_DIR}"/}"
        done | sort
}

# Board targets from sdk/boards: the name comes from board.yml and the
# qualifiers from the <name>_<soc>_<cpu>_defconfig files beside it
list_boards() {
    find "${BOARDS_DIR}" -name board.yml 2>/dev/null | while read -r yml; do
        local dir name soc def rest
        dir="$(dirname "${yml}")"
        name="$(sed -n 's/^[[:space:]]*name:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "${yml}" | head -1)"
        soc="$(sed -n 's/^[[:space:]]*-[[:space:]]*name:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "${yml}" | head -1)"
        [[ -n "${name}" ]] || continue

        for def in "${dir}/${name}"_*_defconfig; do
            [[ -f "${def}" ]] || continue
            rest="$(basename "${def}" _defconfig)"
            rest="${rest#"${name}"_}"
            if [[ -n "${soc}" && "${rest}" == "${soc}"_* ]]; then
                echo "${name}/${soc}/${rest#"${soc}"_}"
            else
                echo "${name}/${rest//_//}"
            fi
        done
    done | sort -u
}

list_ports() {
    ls /dev/ttyACM* /dev/ttyUSB* /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null || true
}

# ============================================================================ #
# Derived values
# ============================================================================ #

app_dir() { echo "${APPS_DIR}/${APP}"; }

# build/<app with / as _>/<board with / as _>
build_dir() {
    local a="${APP//\//_}" b="${BOARD//\//_}"
    echo "${BUILD_ROOT}/${a}/${b}"
}

elf_path() { echo "$(build_dir)/zephyr/zephyr.elf"; }

is_espressif() { [[ "${BOARD}" == esp32* ]]; }

# When run from inside an app directory, pick that app
app_from_cwd() {
    local dir="${PWD}"
    while [[ "${dir}" == "${APPS_DIR}"/* ]]; do
        if [[ -f "${dir}/prj.conf" && -f "${dir}/CMakeLists.txt" ]]; then
            echo "${dir#"${APPS_DIR}"/}"
            return 0
        fi
        dir="$(dirname "${dir}")"
    done
    return 1
}

# ============================================================================ #
# Environment
# ============================================================================ #

check_env() {
    if [[ ! -d "${ZEPHYR_WORKSPACE}/zephyr" ]]; then
        err "No Zephyr workspace at ${ZEPHYR_WORKSPACE}"
        echo "  Run '${0##*/} setup' or set ZEPHYR_WORKSPACE to an existing west workspace."
        return 1
    fi
}

# Run west inside the workspace with its venv, without touching the caller
west_run() {
    check_env || return 1
    (
        cd "${ZEPHYR_WORKSPACE}" || exit 1
        if [[ -f .venv/bin/activate ]]; then
            # shellcheck disable=SC1091
            source .venv/bin/activate
        fi
        echo "${C_DIM}\$ west $*${C_RESET}"
        west "$@"
    )
}

need_app() {
    [[ -n "${APP}" && -d "$(app_dir)" ]] && return 0
    err "No app selected"
    return 1
}

need_board() {
    [[ -n "${BOARD}" ]] && return 0
    err "No board selected"
    return 1
}

need_build() {
    [[ -f "$(elf_path)" ]] && return 0
    err "Nothing built for ${APP} on ${BOARD} yet"
    return 1
}

# Use the remembered port, or the only one connected; empty lets west decide
resolve_port() {
    if [[ -n "${PORT}" && -e "${PORT}" ]]; then
        echo "${PORT}"
        return
    fi
    local ports
    ports="$(list_ports)"
    if [[ -n "${ports}" && "$(echo "${ports}" | wc -l)" -eq 1 ]]; then
        echo "${ports}"
    fi
}

# ============================================================================ #
# Commands
# ============================================================================ #

cmd_build() {
    local pristine="${1:-auto}"
    need_app && need_board || return 1
    info "Building ${C_BOLD}${APP}${C_RESET} for ${C_BOLD}${BOARD}${C_RESET} (pristine=${pristine})"
    local start=${SECONDS}
    if west_run build -b "${BOARD}" -s "$(app_dir)" -d "$(build_dir)" -p "${pristine}"; then
        ok "Built in $((SECONDS - start)) s: $(elf_path)"
    else
        err "Build failed"
        return 1
    fi
}

cmd_flash() {
    need_app && need_board && need_build || return 1
    local port args=()
    port="$(resolve_port)"
    if [[ -n "${port}" ]] && is_espressif; then
        args=(--esp-device "${port}")
    fi
    info "Flashing ${APP} ${port:+on ${port}}"
    west_run flash -d "$(build_dir)" "${args[@]+"${args[@]}"}"
}

cmd_monitor() {
    need_app && need_board || return 1
    local port
    port="$(resolve_port)"
    if is_espressif; then
        local args=()
        [[ -f "$(elf_path)" ]] && args+=(-e "$(elf_path)")
        [[ -n "${port}" ]] && args+=(-p "${port}")
        info "Monitor ${port:+on ${port}} ${C_DIM}(Ctrl+] to exit)${C_RESET}"
        west_run espressif monitor "${args[@]+"${args[@]}"}"
    else
        if [[ -z "${port}" ]]; then
            err "Select a serial port first"
            return 1
        fi
        info "Monitor on ${port} ${C_DIM}(Ctrl+] to exit)${C_RESET}"
        check_env && (
            cd "${ZEPHYR_WORKSPACE}" && source .venv/bin/activate 2>/dev/null
            python3 -m serial.tools.miniterm --raw "${port}" 115200
        )
    fi
}

cmd_run() { cmd_build auto && cmd_flash && cmd_monitor; }

cmd_menuconfig() {
    need_app && need_board || return 1
    [[ -f "$(build_dir)/CMakeCache.txt" ]] || cmd_build auto || return 1
    west_run build -d "$(build_dir)" -t menuconfig
}

cmd_clean() {
    need_app && need_board || return 1
    [[ -d "$(build_dir)" ]] || { ok "Already clean"; return 0; }
    west_run build -d "$(build_dir)" -t clean && ok "Cleaned $(build_dir)"
}

cmd_purge() {
    need_app && need_board || return 1
    local dir
    dir="$(build_dir)"
    [[ -d "${dir}" ]] || { ok "No build directory"; return 0; }
    # Only ever delete our own build/<app>/<board> directories
    case "${dir}" in
        "${BUILD_ROOT}"/*/*) rm -rf "${dir}" && ok "Deleted ${dir}" ;;
        *) err "Refusing to delete ${dir}" ;;
    esac
}

cmd_report() {
    need_app && need_board && need_build || return 1
    local target
    target="$(pick "Report" "ram_report" "rom_report" "footprint (both)")" || return 0
    case "${target}" in
        ram_report | rom_report) west_run build -d "$(build_dir)" -t "${target}" ;;
        *) west_run build -d "$(build_dir)" -t ram_report && west_run build -d "$(build_dir)" -t rom_report ;;
    esac
}

cmd_update() { west_run update; }

cmd_setup() {
    if [[ ! -x "${SETUP_SCRIPT}" && ! -f "${SETUP_SCRIPT}" ]]; then
        err "Setup script not found: ${SETUP_SCRIPT}"
        return 1
    fi
    bash "${SETUP_SCRIPT}" "${ZEPHYR_WORKSPACE}"
}

cmd_status() {
    local state="${C_DIM}not built${C_RESET}"
    if [[ -n "${APP}" && -n "${BOARD}" && -f "$(elf_path)" ]]; then
        local when
        when="$(date -r "$(elf_path)" '+%H:%M:%S %b %d' 2>/dev/null)"
        state="${C_GREEN}built${C_RESET} ${C_DIM}${when}${C_RESET}"
    fi
    row "app" "${APP:-${C_YELLOW}<none>${C_RESET}}"
    row "board" "${BOARD:-${C_YELLOW}<none>${C_RESET}}"
    row "port" "${PORT:-${C_DIM}auto${C_RESET}}"
    row "build" "${state}"
}

# ============================================================================ #
# Selection
# ============================================================================ #

# pick <title> <option>... : numbered menu, prints the chosen option.
# Returns 1 when cancelled with an empty answer or q.
pick() {
    local title="$1"
    shift
    local options=("$@") i choice
    [[ ${#options[@]} -gt 0 ]] || return 1

    {
        echo
        echo "${C_BOLD}${title}${C_RESET}"
        for i in "${!options[@]}"; do
            printf "  ${C_CYAN}%2d${C_RESET}) %s\n" $((i + 1)) "${options[$i]}"
        done
        printf "Choose [1-%d, Enter to cancel]: " ${#options[@]}
    } >&2
    read -r choice
    [[ -z "${choice}" || "${choice}" == q ]] && return 1
    if [[ "${choice}" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#options[@]})); then
        echo "${options[$((choice - 1))]}"
        return 0
    fi
    err "Invalid choice"
    return 1
}

select_app() {
    local apps=() line
    while read -r line; do apps+=("${line}"); done < <(list_apps)
    APP="$(pick "Select app" "${apps[@]}")" || { load_state; return; }
    save_state
}

select_board() {
    local boards=() line choice
    while read -r line; do boards+=("${line}"); done < <(list_boards)
    choice="$(pick "Select board" "${boards[@]}" "other (type a board target)")" || return
    if [[ "${choice}" == other* ]]; then
        printf "Board target (e.g. esp32c6_devkitc/esp32c6/hpcore): " >&2
        read -r choice
        [[ -n "${choice}" ]] || return
    fi
    BOARD="${choice}"
    save_state
}

select_port() {
    local ports=() line choice
    while read -r line; do ports+=("${line}"); done < <(list_ports)
    if [[ ${#ports[@]} -eq 0 ]]; then
        warn "No serial ports found (on WSL, attach the USB device with usbipd)"
    fi
    choice="$(pick "Select serial port" "auto" "${ports[@]+"${ports[@]}"}")" || return
    [[ "${choice}" == auto ]] && PORT="" || PORT="${choice}"
    save_state
}

# ============================================================================ #
# Interactive menu
# ============================================================================ #

menu_item() { printf "  ${C_CYAN}%s${C_RESET}  %-22s" "$1" "$2"; }

show_menu() {
    echo
    echo "${C_BOLD}${C_BLUE}Reiki Zephyr${C_RESET}  ${C_DIM}${ZEPHYR_WORKSPACE}${C_RESET}"
    cmd_status
    echo
    menu_item 1 "Build" && menu_item 2 "Pristine build" && echo
    menu_item 3 "Flash" && menu_item 4 "Monitor" && echo
    menu_item 5 "Build, flash, monitor" && menu_item 6 "Flash and monitor" && echo
    menu_item 7 "Menuconfig" && menu_item 8 "RAM/ROM report" && echo
    menu_item 9 "Clean" && menu_item 0 "Delete build dir" && echo
    echo
    menu_item a "Change app" && menu_item b "Change board" && echo
    menu_item p "Serial port" && menu_item u "West update" && echo
    menu_item s "Setup Zephyr" && menu_item q "Quit" && echo
    echo
    printf "> "
}

# Confirm before something slow or destructive
confirm() {
    local answer
    printf "%s [y/N] " "$1"
    read -r answer
    [[ "${answer}" == y || "${answer}" == Y ]]
}

interactive() {
    [[ -n "${APP}" ]] || select_app
    [[ -n "${BOARD}" ]] || select_board

    while true; do
        show_menu
        local key
        read -r -n 1 key
        echo
        case "${key}" in
            1) cmd_build auto ;;
            2) cmd_build always ;;
            3) cmd_flash ;;
            4) cmd_monitor ;;
            5) cmd_run ;;
            6) cmd_flash && cmd_monitor ;;
            7) cmd_menuconfig ;;
            8) cmd_report ;;
            9) cmd_clean ;;
            0) confirm "Delete $(build_dir)?" && cmd_purge ;;
            a) select_app ;;
            b) select_board ;;
            p) select_port ;;
            u) confirm "Run west update in ${ZEPHYR_WORKSPACE}?" && cmd_update ;;
            s) confirm "Install or update Zephyr in ${ZEPHYR_WORKSPACE}?" && cmd_setup ;;
            q | $'\e') exit 0 ;;
            "") ;;
            *) warn "Unknown option '${key}'" ;;
        esac
    done
}

# ============================================================================ #
# Command line
# ============================================================================ #

usage() {
    cat <<EOF
${C_BOLD}Usage${C_RESET}
  ${0##*/}                         interactive menu
  ${0##*/} <command> [options]     run one command

${C_BOLD}Commands${C_RESET}
  build        incremental build
  pristine     pristine build
  flash        flash the last build
  monitor      serial monitor
  run          build, flash and monitor
  menuconfig   Kconfig menu for the build
  report       RAM/ROM usage
  clean        clean the build directory
  purge        delete the build directory
  apps         list apps
  boards       list board targets from sdk/boards
  ports        list serial ports
  status       show the current selection
  update       west update
  setup        install Zephyr into the workspace

${C_BOLD}Options${C_RESET}
  -a, --app <app>       app under apps/ (e.g. ai_gen/juul); default: the app
                        you are in, else the last one used
  -b, --board <target>  board target (e.g. esp32s3_1_8_amoled_touch/esp32s3/procpu)
  -p, --port <device>   serial port; default: the only connected one

${C_BOLD}Environment${C_RESET}
  ZEPHYR_WORKSPACE      west workspace (default ~/zephyrproject)

Builds go to build/<app>/<board>/. Selections are remembered in .zephyr_cache.
EOF
}

main() {
    local command="" cwd_app
    load_state
    if cwd_app="$(app_from_cwd)"; then
        APP="${cwd_app}"
    fi

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -a | --app)
                APP="${2%/}"
                APP="${APP#apps/}"
                shift 2
                ;;
            -b | --board)
                BOARD="$2"
                shift 2
                ;;
            -p | --port)
                PORT="$2"
                shift 2
                ;;
            -h | --help | help)
                usage
                exit 0
                ;;
            -*)
                err "Unknown option $1"
                usage
                exit 2
                ;;
            *)
                command="$1"
                shift
                ;;
        esac
    done

    if [[ -n "${APP}" && ! -d "$(app_dir)" ]]; then
        err "No app at apps/${APP}"
        exit 1
    fi
    save_state

    case "${command}" in
        "") interactive ;;
        build) cmd_build auto ;;
        pristine) cmd_build always ;;
        flash) cmd_flash ;;
        monitor) cmd_monitor ;;
        run) cmd_run ;;
        menuconfig) cmd_menuconfig ;;
        report) cmd_report ;;
        clean) cmd_clean ;;
        purge) cmd_purge ;;
        apps) list_apps ;;
        boards) list_boards ;;
        ports) list_ports ;;
        status) cmd_status ;;
        update) cmd_update ;;
        setup) cmd_setup ;;
        *)
            err "Unknown command '${command}'"
            usage
            exit 2
            ;;
    esac
}

main "$@"
