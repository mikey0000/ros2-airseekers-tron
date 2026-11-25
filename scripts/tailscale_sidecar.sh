#!/usr/bin/env bash
# The owner's Tailscale node on the mower host (container mowgli-remote), kept up to date by
# image, independent of this stack and of the vendor software (survives switch_stack.sh in both
# directions). Run ON THE MOWER HOST as root:
#
#   sudo scripts/tailscale_sidecar.sh install   # copy this script to /usr/local/sbin, (re)create
#                                               # the container from the spec below, daily timer
#   sudo tailscale_sidecar.sh update            # pull the image; recreate only when it changed
#   sudo tailscale_sidecar.sh status
#   sudo tailscale_sidecar.sh uninstall         # timer + script removed; container left running
#
# The spec matches the GUI's remote-access sidecar (MowgliNext remote_access.go buildSpec):
# userspace networking (tailnet ports reach the same port on localhost, e.g. ssh 22), host
# network, every capability dropped, state in the docker volume mowgli_remote_state, so a
# recreate keeps the node identity, its name and its preferences. Tailscale's own
# `--auto-update` cannot replace the binary inside this container (read-only /usr/local/bin
# without capabilities): docker installs are updated by image, which is what `update` does.
# If the ROS 2 stack's GUI runs with Remote access enabled it may recreate the container with
# its own pinned image; the next `update` run brings it back to this spec.
#
# A recreate never leaves the host without a node: the old container is stopped and kept
# (mowgli-remote-old) until the new one reports BackendState Running, else it is put back.
# The swap runs as a transient systemd unit, so it finishes even when the shell that started
# it was a Tailscale SSH session carried by the container being replaced (its log:
# journalctl -u tailscale-sidecar-update-swap).
set -euo pipefail

NAME=mowgli-remote
IMAGE=${TS_IMAGE:-tailscale/tailscale:stable}
VOLUME=mowgli_remote_state
HOSTNAME_TS=${TS_HOSTNAME:-mowgli}
OLD=$NAME-old
SELF=/usr/local/sbin/tailscale_sidecar.sh
UNIT=tailscale-sidecar-update

[[ $EUID -eq 0 ]] || { echo "run as root (sudo)" >&2; exit 1; }

run_container() {
    docker run -d --name "$1" \
        --network host --restart unless-stopped --cap-drop ALL \
        -v "$VOLUME:/var/lib/tailscale" \
        -e TS_HOSTNAME="$HOSTNAME_TS" -e TS_STATE_DIR=/var/lib/tailscale \
        -e TS_USERSPACE=true -e TS_AUTH_ONCE=true -e TS_ACCEPT_DNS=false \
        --label project=mowglinext --label app=remote-access \
        --label com.centurylinklabs.watchtower.enable=false \
        "$IMAGE" >/dev/null
}

running() {
    local _
    for _ in $(seq 30); do
        docker exec "$1" tailscale status --json 2>/dev/null |
            grep -Eq '"BackendState": *"Running"' && return 0
        sleep 2
    done
    return 1
}

swap() {
    if ! docker inspect "$NAME" >/dev/null 2>&1; then
        run_container "$NAME"            # first install: may need a login (NeedsLogin), no rollback
    else
        docker rm -f "$OLD" >/dev/null 2>&1 || true
        docker rename "$NAME" "$OLD"
        docker stop "$OLD" >/dev/null
        if ! { run_container "$NAME" && running "$NAME"; }; then
            docker rm -f "$NAME" >/dev/null 2>&1 || true
            docker rename "$OLD" "$NAME"
            docker start "$NAME" >/dev/null
            echo "new $NAME did not reach Running in 60 s: the old container is back" >&2
            return 1
        fi
        docker rm "$OLD" >/dev/null
    fi
    echo "$NAME (re)created from $IMAGE ($(docker image inspect -f '{{.Id}}' "$IMAGE" | cut -c8-19))"
}

create() {
    if [[ -n ${INVOCATION_ID:-} ]]; then  # already a systemd unit (the timer)
        swap
    else
        local rc=0
        systemd-run --quiet --wait --collect --unit=$UNIT-swap "$(realpath "$0")" _swap || rc=$?
        journalctl -u $UNIT-swap -n 3 --no-pager -o cat
        return $rc
    fi
}

update() {
    docker pull -q "$IMAGE" >/dev/null
    local want have
    want=$(docker image inspect -f '{{.Id}}' "$IMAGE")
    have=$(docker inspect -f '{{.Image}}' "$NAME" 2>/dev/null || true)
    if [[ $want == "$have" ]]; then
        docker start "$NAME" >/dev/null      # no-op when running
        echo "$NAME up to date ($(docker exec "$NAME" tailscale version | head -1))"
    else
        create
        [[ -n $have ]] && docker image rm "$have" >/dev/null 2>&1 || true
    fi
}

install_timer() {
    [[ $(realpath "$0") == "$SELF" ]] || install -m 0755 "$0" "$SELF"
    cat > /etc/systemd/system/$UNIT.service <<EOF
[Unit]
Description=Update the Tailscale container $NAME by image
After=docker.service network-online.target
Requires=docker.service

[Service]
Type=oneshot
ExecStart=$SELF update
EOF
    cat > /etc/systemd/system/$UNIT.timer <<EOF
[Unit]
Description=Daily image update of $NAME

[Timer]
OnCalendar=*-*-* 04:30:00
RandomizedDelaySec=1h
Persistent=true

[Install]
WantedBy=timers.target
EOF
    systemctl daemon-reload
    systemctl enable --now $UNIT.timer
}

case ${1:-} in
    install)   install_timer; docker pull -q "$IMAGE" >/dev/null; create ;;
    update)    update ;;
    _swap)     swap ;;
    status)    docker ps -a -f name="^$NAME\$" --format '{{.Names}} {{.Image}} {{.Status}}'
               docker exec "$NAME" tailscale version 2>/dev/null | head -1 || true
               systemctl list-timers --no-pager $UNIT.timer 2>/dev/null | sed -n 1,2p ;;
    uninstall) systemctl disable --now $UNIT.timer 2>/dev/null || true
               rm -f /etc/systemd/system/$UNIT.{service,timer} "$SELF"
               systemctl daemon-reload; echo "timer removed; $NAME left running" ;;
    *) sed -n 2,11p "$0"; exit 2 ;;
esac
