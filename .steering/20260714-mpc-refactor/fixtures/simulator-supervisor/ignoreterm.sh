#!/usr/bin/env bash

if [[ -n ${FAKE_SIMULATOR_PID_FILE:-} ]]; then
    printf '%s\n' "${BASHPID}" >"${FAKE_SIMULATOR_PID_FILE}"
fi
trap '' INT TERM
while true; do
    sleep 0.05
done
