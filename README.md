# crazyflie_simulation

Hi! Welkom to the crazyflie simulation repo. This repo is still at the early stages and still in development so just ask if you are looking for anything in particular.

Currently it contains low resolution meshes based on the design of the actual [crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) but heavily simplified to contain as little vertices as possible, so it can be used to simulate a lot of them.


## Installation

See the  [installation instructions](/docs/installing/) in the Github docs folder.

## Official documentation

Check out the [Bitcraze crazyflie-simulation documentation](https://www.bitcraze.io/documentation/repository/crazyflie-simulation/main/) on our website.

## Contribute

Go to the [contribute page](https://www.bitcraze.io/development/contribute/) on our website to learn more.

### Matching the Gazebo IMU rate to SITL firmware

The Gazebo launch scripts read `CRAZYSIM_RATE_MAIN_LOOP` from the SITL build's
`CMakeCache.txt` and use that value for the generated model IMU rate. This
keeps the sensor cadence aligned when the firmware is built at 500 Hz; builds
without that cache setting retain the 1000 Hz default. Set
`NCS_CRAZYSIM_IMU_RATE=500` or `1000` only to override the detected rate
deliberately. The generator rejects rates other than those supported values.

### Bounded firmware motor-packet trace

To capture packets received from the firmware, set
`NCS_CRAZYSIM_TRACE_MOTOR=all` and `NCS_CRAZYSIM_TRACE_MOTOR_PATH` to a new
output file before starting Gazebo. The plugin stores up to 32,768 records in
memory and writes JSONL on shutdown. Each record includes the host monotonic
arrival time, all nine received packet bytes, four decoded PWM ratios, and the
four converted motor speeds in rad/s. Additional packets after capacity are
counted and dropped. This mode avoids per-packet console output, but still adds
bounded memory and receive-thread work; compare it with a tracing-disabled
run before using the measurements for timing claims.
