#!/usr/bin/env bash
# Pi companion identity from /etc/default/companion-drone.
#
# companion_file_id <defaults-file>
#   Prints an id in 1..9 with no trailing newline, or nothing.
#   The first COMPANION_DRONE_ID assignment decides. Blank lines and
#   comments are skipped. 0, 10+, and non-integers print nothing.
#
# companion_windows_to_drop <keep_id>
#   Reads tmux window names on stdin. Prints hardware_adapter_<n> and
#   vio_<n> whose integer suffix is not keep_id, one name per line.

companion_file_id() {
    local file="${1:-}"
    local line key val n
    [ -n "${file}" ] && [ -f "${file}" ] && [ -r "${file}" ] || return 0
    while IFS= read -r line || [ -n "${line}" ]; do
        line="${line%"${line##*[![:space:]]}"}"
        line="${line#"${line%%[![:space:]]*}"}"
        [ -n "${line}" ] || continue
        case "${line}" in
            \#*) continue ;;
        esac
        case "${line}" in
            *=*) ;;
            *) continue ;;
        esac
        key="${line%%=*}"
        val="${line#*=}"
        key="${key%"${key##*[![:space:]]}"}"
        key="${key#"${key%%[![:space:]]*}"}"
        [ "${key}" = "COMPANION_DRONE_ID" ] || continue
        val="${val%"${val##*[![:space:]]}"}"
        val="${val#"${val%%[![:space:]]*}"}"
        case "${val}" in
            \"*\") val="${val#\"}"; val="${val%\"}" ;;
            \'*\') val="${val#\'}"; val="${val%\'}" ;;
        esac
        if [[ "${val}" =~ ^[0-9]+$ ]]; then
            n=$((10#${val}))
            if [ "${n}" -ge 1 ] && [ "${n}" -le 9 ]; then
                printf '%s' "${n}"
            fi
        fi
        return 0
    done < "${file}"
    return 0
}

companion_windows_to_drop() {
    local keep_id="${1:-}"
    local name suffix
    while IFS= read -r name || [ -n "${name}" ]; do
        case "${name}" in
            hardware_adapter_*) suffix="${name#hardware_adapter_}" ;;
            vio_*) suffix="${name#vio_}" ;;
            *) continue ;;
        esac
        [[ "${suffix}" =~ ^[0-9]+$ ]] || continue
        if [[ "${keep_id}" =~ ^[0-9]+$ ]] && [ "$((10#${suffix}))" -eq "$((10#${keep_id}))" ]; then
            continue
        fi
        printf '%s\n' "${name}"
    done
}
