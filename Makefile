# Runs the swarm's pieces together (scripts/up.sh); Ctrl-C, or closing the viewer, stops them
# all. Output goes to the terminal, each line prefixed with who wrote it, and to logs/.
#
#   make                 backend + viewer + ground station
#   make run ARGS="--mode connected --field 130x30"
#   make hil             flash the board, then fly it as drone 1 (FLASH=0: as it is;
#                        PORT=/dev/tty.usbmodem...: the board's port, if not the only one)
#   make fake            the host-side fake board as drone 1
#   make flash           just flash the board and watch its log
#   make obs | obs-down  Grafana/Prometheus/Tempo (http://localhost:3000)
#   make build | test | smoke

export PATH := $(HOME)/.cargo/bin:$(PATH)
export ARGS ?=
export PORT ?=
export FLASH ?= 1

.PHONY: run hil fake flash obs obs-down build test smoke

# Built first, so the windows don't open one by one while each crate compiles.
run: build
	@scripts/up.sh run

hil: build
	@scripts/up.sh hil

fake: build
	@scripts/up.sh fake

flash:
	cd firmware && cargo run --release -- --no-catch-reset

obs:
	cd observability && docker compose up -d

obs-down:
	cd observability && docker compose down

build:
	cd backend && cargo build && cargo build --examples
	cd viz && cargo build --release
	cd gcs && cargo build --release

test:
	cd brain && cargo test --features std
	cd backend && cargo test
	cd gcs && cargo test

# The ground station's and the remote's protocol checks, each against its own backend.
smoke: build
	@scripts/up.sh smoke-gcs
	@scripts/up.sh smoke-remote
