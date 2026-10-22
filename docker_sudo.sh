# shellcheck shell=bash
# Resolve `docker` vs `sudo docker` when unix:///var/run/docker.sock is root-only.
# Source this file, then call catswarm_docker instead of docker.
#
# CATSWARM_DOCKER_SUDO=0  never sudo
# CATSWARM_DOCKER_SUDO=1  always sudo docker (GUI backend sets this after a failed probe)
# unset                   try docker info; sudo only on permission denied

catswarm_docker_init() {
    case "${CATSWARM_DOCKER_SUDO:-}" in
        0|false|no|off)
            CATSWARM_DOCKER_SUDO=0
            return 0
            ;;
        1|true|yes|on)
            CATSWARM_DOCKER_SUDO=1
            ;;
        *)
            if docker info >/dev/null 2>&1; then
                CATSWARM_DOCKER_SUDO=0
                return 0
            fi
            local _docker_err
            _docker_err="$(docker info 2>&1 || true)"
            if printf '%s\n' "$_docker_err" | grep -qiE 'permission denied|docker[.]sock'; then
                echo "Docker socket is not writable; retrying with sudo docker."
                CATSWARM_DOCKER_SUDO=1
            else
                CATSWARM_DOCKER_SUDO=0
                return 0
            fi
            ;;
    esac
    if [ "${CATSWARM_DOCKER_SUDO}" = "1" ] && [ -t 0 ]; then
        echo "If sudo asks for a password, type it in this terminal."
        sudo -v
    fi
}

catswarm_docker() {
    catswarm_docker_init
    if [ "${CATSWARM_DOCKER_SUDO}" = "1" ]; then
        if [ -t 0 ]; then
            sudo docker "$@"
        else
            sudo -n docker "$@"
        fi
    else
        docker "$@"
    fi
}
