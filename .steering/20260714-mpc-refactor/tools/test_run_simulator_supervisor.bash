#!/usr/bin/env bash

set -u

repo_root="$(cd "$(dirname "$0")/../../../" && pwd)"
runner="${repo_root}/aichallenge/run_simulator.bash"
fixture_dir="${repo_root}/.steering/20260714-mpc-refactor/fixtures/simulator-supervisor"
test_root="$(mktemp -d /tmp/phase0-simulator-supervisor.XXXXXX)"

ros2()
{
    if [[ -n ${FAKE_MANAGER_PID_FILE:-} ]]; then
        printf '%s\n' "${BASHPID}" >"${FAKE_MANAGER_PID_FILE}"
    fi
    if [[ ${FAKE_MANAGER_MODE:-stay} == "fail" ]]; then
        return 9
    fi
    if [[ ${FAKE_MANAGER_MODE:-stay} == "delayed_fail" ]]; then
        sleep 0.2
        return 9
    fi
    if [[ ${FAKE_MANAGER_MODE:-stay} == "clean_exit" ]]; then
        return 0
    fi
    trap 'exit 0' INT TERM
    while true; do
        sleep 0.05
    done
}
export -f ros2

run_case()
{
    local name="$1"
    local expected="$2"
    local manager_enabled="$3"
    local manager_mode="$4"
    local simulator_mode="$5"
    local log_dir="${test_root}/${name}"
    local manager_pid_file="${log_dir}/manager.pid"
    local simulator_pid_file="${log_dir}/simulator.pid"
    local actual observed_pid

    mkdir -p "${log_dir}"
    timeout 5s env \
        AICHALLENGE_SIMULATOR_SCRIPT_DIR="${fixture_dir}" \
        START_AWSIM_STATE_MANAGER="${manager_enabled}" \
        FAKE_MANAGER_MODE="${manager_mode}" \
        FAKE_MANAGER_PID_FILE="${manager_pid_file}" \
        FAKE_SIMULATOR_PID_FILE="${simulator_pid_file}" \
        AICHALLENGE_SHUTDOWN_POLL_INTERVAL=0.02 \
        AICHALLENGE_SHUTDOWN_POLL_ATTEMPTS=10 \
        LOG_DIR="${log_dir}" \
        bash "${runner}" "${simulator_mode}"
    actual=$?
    if [[ ${actual} -ne ${expected} ]]; then
        echo "[FAIL] ${name}: expected=${expected} actual=${actual}" >&2
        return 1
    fi
    if [[ ${manager_enabled} == true ]]; then
        if [[ ! -s ${manager_pid_file} ]]; then
            echo "[FAIL] ${name}: manager PID was not recorded" >&2
            return 1
        fi
        observed_pid="$(<"${manager_pid_file}")"
        if kill -0 -- "-${observed_pid}" 2>/dev/null; then
            kill -KILL -- "-${observed_pid}" 2>/dev/null || true
            echo "[FAIL] ${name}: manager process group survived" >&2
            return 1
        fi
    fi
    if [[ -s ${simulator_pid_file} ]]; then
        observed_pid="$(<"${simulator_pid_file}")"
        if kill -0 -- "-${observed_pid}" 2>/dev/null; then
            kill -KILL -- "-${observed_pid}" 2>/dev/null || true
            echo "[FAIL] ${name}: simulator process group survived" >&2
            return 1
        fi
    fi
    echo "[PASS] ${name}: status=${actual}"
}

run_term_case()
{
    local name="external_term_is_normalized"
    local log_dir="${test_root}/${name}"
    local actual

    mkdir -p "${log_dir}"
    timeout --preserve-status --signal=TERM --kill-after=2s 0.2s env \
        AICHALLENGE_SIMULATOR_SCRIPT_DIR="${fixture_dir}" \
        START_AWSIM_STATE_MANAGER=true \
        FAKE_MANAGER_MODE=stay \
        AICHALLENGE_SHUTDOWN_POLL_INTERVAL=0.02 \
        AICHALLENGE_SHUTDOWN_POLL_ATTEMPTS=10 \
        LOG_DIR="${log_dir}" \
        bash "${runner}" sleep
    actual=$?
    if [[ ${actual} -ne 143 ]]; then
        echo "[FAIL] ${name}: expected=143 actual=${actual}" >&2
        return 1
    fi
    echo "[PASS] ${name}: status=${actual}"
}

run_eval_default_case()
{
    local name="eval_defaults_to_external_manager_owner"
    local log_dir="${test_root}/${name}"
    local manager_pid_file="${log_dir}/manager.pid"
    local actual

    mkdir -p "${log_dir}"
    env -u START_AWSIM_STATE_MANAGER \
        AICHALLENGE_SIMULATOR_SCRIPT_DIR="${fixture_dir}" \
        FAKE_MANAGER_MODE=stay \
        FAKE_MANAGER_PID_FILE="${manager_pid_file}" \
        LOG_DIR="${log_dir}" \
        bash "${runner}" eval
    actual=$?
    if [[ ${actual} -ne 7 ]]; then
        echo "[FAIL] ${name}: expected=7 actual=${actual}" >&2
        return 1
    fi
    if [[ -e ${manager_pid_file} ]]; then
        echo "[FAIL] ${name}: runner unexpectedly started the manager" >&2
        return 1
    fi
    echo "[PASS] ${name}: status=${actual}, manager owned by evaluation launch"
}

run_killed_manager_case()
{
    local name="manager_wrapper_sigkill_is_detected"
    local log_dir="${test_root}/${name}"
    local manager_pid_file="${log_dir}/manager.pid"
    local case_pid manager_pid actual attempt

    mkdir -p "${log_dir}"
    timeout --preserve-status --signal=TERM --kill-after=2s 5s env \
        AICHALLENGE_SIMULATOR_SCRIPT_DIR="${fixture_dir}" \
        START_AWSIM_STATE_MANAGER=true \
        FAKE_MANAGER_MODE=stay \
        FAKE_MANAGER_PID_FILE="${manager_pid_file}" \
        AICHALLENGE_SHUTDOWN_POLL_INTERVAL=0.02 \
        AICHALLENGE_SHUTDOWN_POLL_ATTEMPTS=10 \
        LOG_DIR="${log_dir}" \
        bash "${runner}" sleep &
    case_pid=$!

    for ((attempt = 0; attempt < 100; ++attempt)); do
        [[ -s ${manager_pid_file} ]] && break
        sleep 0.01
    done
    if [[ ! -s ${manager_pid_file} ]]; then
        kill -TERM "${case_pid}" 2>/dev/null || true
        wait "${case_pid}" 2>/dev/null || true
        echo "[FAIL] ${name}: manager PID was not observed" >&2
        return 1
    fi

    manager_pid="$(<"${manager_pid_file}")"
    kill -KILL "${manager_pid}"
    wait "${case_pid}"
    actual=$?
    if [[ ${actual} -ne 137 ]]; then
        echo "[FAIL] ${name}: expected=137 actual=${actual}" >&2
        return 1
    fi
    if kill -0 -- "-${manager_pid}" 2>/dev/null; then
        kill -KILL -- "-${manager_pid}" 2>/dev/null || true
        echo "[FAIL] ${name}: manager process group survived runner exit" >&2
        return 1
    fi
    echo "[PASS] ${name}: status=${actual}, no orphaned manager group"
}

failures=0
run_case simulator_fast_failure 7 false stay exit7 || failures=$((failures + 1))
run_case manager_fast_failure 9 true fail sleep || failures=$((failures + 1))
run_case manager_clean_early_exit 1 true clean_exit sleep || failures=$((failures + 1))
run_case simulator_failure_stops_manager 7 true stay exit7 || failures=$((failures + 1))
run_case manager_failure_kills_unresponsive_simulator 9 true delayed_fail ignoreterm || failures=$((failures + 1))
run_eval_default_case || failures=$((failures + 1))
run_term_case || failures=$((failures + 1))
run_killed_manager_case || failures=$((failures + 1))

if [[ ${failures} -ne 0 ]]; then
    echo "${failures} supervisor test(s) failed; logs: ${test_root}" >&2
    exit 1
fi
echo "all supervisor tests passed; logs: ${test_root}"
