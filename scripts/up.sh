#!/usr/bin/env bash
# Starts a set of the swarm's processes together (see the Makefile) and stops them together:
# on Ctrl-C, or as soon as any one of them exits (closing the viewer's window stops the rest).
# Each one's output goes to the terminal, prefixed with its name, and to logs/<name>.log.
#
#   scripts/up.sh run            backend + viewer + ground station
#   scripts/up.sh hil            the same, with the board (flashed first unless FLASH=0) as drone 1
#   scripts/up.sh fake           the same, with the host-side fake board as drone 1
#   scripts/up.sh smoke-gcs      backend + examples/gcs_smoke (exits with the check's status)
#   scripts/up.sh smoke-remote   backend --mode connected + examples/remote_smoke
#
# ARGS: more backend flags (e.g. "--mode connected --field 130x30"); PORT: the board's serial port.

set -u
cd "$(dirname "$0")/.."
mkdir -p logs

ARGS=${ARGS:-}
# The board's port is named after the USB socket it's in, so without PORT, the one there is.
if [ -z "${PORT:-}" ]; then
    ports=(/dev/tty.usbmodem*)
    if [ "${#ports[@]}" -eq 1 ] && [ -e "${ports[0]}" ]; then PORT=${ports[0]}; fi
fi
PORT=${PORT:-/dev/tty.usbmodem*}
FLASH=${FLASH:-1}

names=()
pids=()

stop() {
    trap - INT TERM EXIT
    for pid in "${pids[@]+"${pids[@]}"}"; do kill "$pid" 2>/dev/null; done
    wait 2>/dev/null
}
trap stop EXIT
trap 'exit 130' INT TERM

# start <name> <dir> <command...>
start() {
    local name=$1 dir=$2
    shift 2
    echo "==> $name: (cd $dir && $*)"
    (cd "$dir" && exec "$@") > >(tee "logs/$name.log" | awk -v p="[$name] " '{ print p $0; fflush() }') 2>&1 &
    names+=("$name")
    pids+=($!)
}

# Refuses to start over something already running: a second backend can't bind its ports, and
# its drones would talk to the first one's.
free_tcp() {
    if lsof -nP -iTCP:"$1" -sTCP:LISTEN >/dev/null 2>&1; then
        echo "TCP port $1 is taken (an earlier run still going?):" >&2
        lsof -nP -iTCP:"$1" -sTCP:LISTEN >&2
        exit 1
    fi
}
free_udp() {
    if lsof -nP -iUDP:"$1" >/dev/null 2>&1; then
        echo "UDP port $1 is taken (an earlier run still going?):" >&2
        lsof -nP -iUDP:"$1" >&2
        exit 1
    fi
}
backend_ports_free() {
    free_tcp 9002   # viewer feed
    free_udp 14600  # radio emulator
    free_udp 14650  # base
}

# Waits until the last process started has written a line matching <pattern> to its log, or
# gives up after <seconds> (or as soon as it exits).
wait_for_log() {
    local pattern=$1 seconds=$2 name=${names[${#names[@]}-1]} pid=${pids[${#pids[@]}-1]}
    for _ in $(seq "$seconds"); do
        grep -q -- "$pattern" "logs/$name.log" 2>/dev/null && return 0
        kill -0 "$pid" 2>/dev/null || { echo "$name exited before it was ready" >&2; exit 1; }
        sleep 1
    done
    echo "$name: no \"$pattern\" in ${seconds}s" >&2
    exit 1
}

# Waits until the backend listens for the viewer (it's up), up to a minute.
wait_for_backend() {
    local pid=${pids[${#pids[@]}-1]}
    for _ in $(seq 60); do
        lsof -nP -iTCP:9002 -sTCP:LISTEN >/dev/null 2>&1 && return 0
        kill -0 "$pid" 2>/dev/null || { echo "backend exited before it was up" >&2; exit 1; }
        sleep 1
    done
    echo "backend isn't listening on 9002 after 60s" >&2
    exit 1
}

# Until one of them exits; then its status is ours (stop() takes down the rest).
wait_any() {
    while true; do
        for i in "${!pids[@]}"; do
            if ! kill -0 "${pids[$i]}" 2>/dev/null; then
                wait "${pids[$i]}"
                local status=$?
                echo "==> ${names[$i]} exited ($status), stopping the rest"
                exit "$status"
            fi
        done
        sleep 1
    done
}

viewers() {
    start viz viz cargo run --release
    start gcs gcs cargo run --release
}

# shellcheck disable=SC2086 # ARGS is a list of flags
case ${1:-run} in
    run)
        backend_ports_free
        start backend backend cargo run -- $ARGS
        wait_for_backend
        viewers
        ;;
    hil)
        backend_ports_free
        [ -e "$PORT" ] || { echo "no board at $PORT (set PORT=...)" >&2; exit 1; }
        if lsof "$PORT" >/dev/null 2>&1; then
            echo "$PORT is open in another program:" >&2
            lsof "$PORT" >&2
            exit 1
        fi
        if [ "$FLASH" = 1 ]; then
            start firmware firmware cargo run --release -- --no-catch-reset
            # probe-rs says so once the board is flashed and running: "Finished in", in color, so
            # with escape codes between. Not cargo's "Finished `release` profile ... in".
            wait_for_log 'Finished[^`]* in ' 120
        fi
        start backend backend cargo run -- --hil "$PORT" $ARGS
        wait_for_backend
        viewers
        ;;
    fake)
        backend_ports_free
        free_tcp 5760
        start backend backend cargo run -- --hil tcpin:127.0.0.1:5760 $ARGS
        wait_for_backend
        start fake_board backend cargo run --example fake_board
        viewers
        ;;
    smoke-gcs)
        backend_ports_free
        start backend backend cargo run -- $ARGS
        wait_for_backend
        start gcs_smoke backend cargo run --example gcs_smoke
        ;;
    smoke-remote)
        backend_ports_free
        start backend backend cargo run -- --mode connected $ARGS
        wait_for_backend
        start remote_smoke backend cargo run --example remote_smoke
        ;;
    *)
        echo "unknown set: $1 (run, hil, fake, smoke-gcs, smoke-remote)" >&2
        exit 2
        ;;
esac

wait_any
