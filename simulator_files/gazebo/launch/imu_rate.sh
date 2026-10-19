#!/usr/bin/env bash

resolve_sitl_imu_rate() {
	local build_path="$1"
	local configured_rate=""

	if [[ -n "${NCS_CRAZYSIM_IMU_RATE:-}" ]]; then
		printf '%s\n' "$NCS_CRAZYSIM_IMU_RATE"
		return
	fi

	if [[ -f "$build_path/CMakeCache.txt" ]]; then
		configured_rate="$(sed -n 's/^CRAZYSIM_RATE_MAIN_LOOP:STRING=//p' "$build_path/CMakeCache.txt")"
	fi

	printf '%s\n' "${configured_rate:-1000}"
}
