#!/usr/bin/env bash

SCRIPT_DIR="${AICHALLENGE_SIMULATOR_SCRIPT_DIR:-$(cd "$(dirname "$0")" && pwd)/simulator_scripts}"
mode="${1:-${SIM_MODE:-simulator}}"
[ $# -gt 0 ] && shift

# dev2 / gate2 等は dev.sh / gate.sh に番号を渡すエイリアス
[[ ${mode} =~ ^(dev|gate)([0-9]+)$ ]] && set -- "${BASH_REMATCH[2]}" "$@" && mode="${BASH_REMATCH[1]}"

# simulator_scripts 内のスクリプトを呼び出す
script="${SCRIPT_DIR}/${mode}.sh"
if [[ ! -f ${script} ]]; then
    echo "[ERROR] unknown mode '${mode}' (supported: $(basename -s .sh "${SCRIPT_DIR}"/*.sh | xargs) dev<N> gate<N>)" >&2
    exit 1
fi

log_dir="${LOG_DIR:-/output}"
mkdir -p "${log_dir}"
log_dir="$(cd "${log_dir}" && pwd)" || exit 1
cd "${log_dir}" || exit 1
exec >"${log_dir}/awsim.log" 2>&1

manager_pid=""
simulator_pid=""
termination_signal=""
shutdown_poll_interval="${AICHALLENGE_SHUTDOWN_POLL_INTERVAL:-0.05}"
shutdown_poll_attempts="${AICHALLENGE_SHUTDOWN_POLL_ATTEMPTS:-100}"
start_state_manager="${START_AWSIM_STATE_MANAGER:-}"

if [[ -z ${start_state_manager} ]]; then
    [[ ${mode} == "eval" ]] && start_state_manager=false || start_state_manager=true
fi

if [[ ! ${shutdown_poll_attempts} =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] AICHALLENGE_SHUTDOWN_POLL_ATTEMPTS must be a positive integer" >&2
    exit 1
fi

signal_job()
{
    local pid="$1"
    local signal="$2"
    if [[ -n ${pid} ]]; then
        # Try the process group even when its leader has already exited: a
        # ros2/Unity descendant can still be alive in the same group.
        kill -"${signal}" -- "-${pid}" 2>/dev/null || \
            kill -"${signal}" "${pid}" 2>/dev/null || true
    fi
}

forward_signal()
{
    termination_signal="$1"
    signal_job "${simulator_pid}" "${termination_signal}"
    signal_job "${manager_pid}" "${termination_signal}"
}

job_group_alive()
{
    local pid="$1"
    [[ -n ${pid} ]] && kill -0 -- "-${pid}" 2>/dev/null
}

wait_for_pid_exit()
{
    local pid="$1"
    local status
    while true; do
        # Always wait at least once. Bash retains an unwaited fast child's status
        # even after kill -0 has already become false.
        wait -f "${pid}" 2>/dev/null
        status=$?
        if ! kill -0 "${pid}" 2>/dev/null; then
            return "${status}"
        fi
    done
}

stop_job()
{
    local pid="$1"
    local signal="${2:-TERM}"
    local attempt

    if [[ -z ${pid} ]]; then
        return 0
    fi

    signal_job "${pid}" "${signal}"
    for ((attempt = 0; attempt < shutdown_poll_attempts; ++attempt)); do
        if ! job_group_alive "${pid}"; then
            break
        fi
        sleep "${shutdown_poll_interval}"
    done
    if job_group_alive "${pid}"; then
        echo "[WARN] process group ${pid} ignored ${signal}; sending KILL" >&2
        signal_job "${pid}" KILL
    fi

    wait_for_pid_exit "${pid}"
}

trap 'forward_signal TERM' TERM
trap 'forward_signal INT' INT

# Job control prevents background children from inheriting SIGINT as ignored.
set -m

if [[ ${start_state_manager} == "true" ]]; then
    echo "[INFO] Starting awsim_state_manager on ROS_DOMAIN_ID=0"
    (
        export ROS_DOMAIN_ID=0
        ros2 launch \
            aichallenge_system_launch awsim_state_manager.launch.xml
    ) >"${log_dir}/awsim_state_manager.log" 2>&1 &
    manager_pid=$!
fi

echo "[INFO] Starting AWSIM: ${mode}.sh $*"
(
    export ROS_DOMAIN_ID=0
    exec bash "${script}" "$@"
) &
simulator_pid=$!

# Poll both direct children from the parent. This also detects a manager killed
# before it can run shell cleanup/notification code, while explicit wait keeps
# fast-child statuses available on Bash 5.1.
while true; do
    if [[ -n ${termination_signal} ]]; then
        stop_job "${simulator_pid}" "${termination_signal}"
        simulator_pid=""
        stop_job "${manager_pid}" "${termination_signal}"
        manager_pid=""
        [[ ${termination_signal} == "INT" ]] && exit 130
        exit 143
    fi

    if [[ -n ${manager_pid} ]] && ! kill -0 "${manager_pid}" 2>/dev/null; then
        failed_manager_pid="${manager_pid}"
        wait_for_pid_exit "${failed_manager_pid}"
        manager_status=$?
        manager_pid=""
        # The wrapper can be gone while ros2 descendants remain in its PGID.
        stop_job "${failed_manager_pid}" TERM
        echo "[ERROR] awsim_state_manager exited before AWSIM (status=${manager_status})" >&2
        stop_job "${simulator_pid}" TERM
        simulator_pid=""
        [[ ${manager_status} -ne 0 ]] && exit "${manager_status}"
        exit 1
    fi

    if ! kill -0 "${simulator_pid}" 2>/dev/null; then
        finished_simulator_pid="${simulator_pid}"
        wait_for_pid_exit "${finished_simulator_pid}"
        simulator_status=$?
        simulator_pid=""
        # Clean up any descendant that outlived the simulator script leader.
        stop_job "${finished_simulator_pid}" TERM
        if [[ -n ${manager_pid} ]]; then
            stop_job "${manager_pid}" TERM
            manager_pid=""
        fi
        exit "${simulator_status}"
    fi

    sleep 0.02
done
