import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

GAZEBO_DIR = Path(__file__).resolve().parents[1]
GENERATOR = GAZEBO_DIR / "launch" / "jinja_gen.py"


class ImuRateGenerationTest(unittest.TestCase):
    def render(self, model: str, *extra_args: str) -> str:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "model.sdf"
            template = GAZEBO_DIR / "models" / model / "model.sdf.jinja"
            subprocess.run(
                [
                    "uv",
                    "run",
                    "--with",
                    "jinja2",
                    "--with",
                    "numpy",
                    "--",
                    "python",
                    str(GENERATOR),
                    str(template),
                    str(GAZEBO_DIR),
                    "--output-file",
                    str(output),
                    *extra_args,
                ],
                check=True,
                cwd=GAZEBO_DIR,
                capture_output=True,
                text=True,
            )
            return output.read_text()

    def imu_rate(self, sdf: str) -> float:
        root = ET.fromstring(sdf)
        imu = root.find(".//sensor[@name='imu_sensor']/update_rate")
        self.assertIsNotNone(imu)
        assert imu is not None
        return float(imu.text or "nan")

    def test_both_models_default_to_one_kilohertz(self) -> None:
        for model in ("crazyflie", "crazyflie_thrust_upgrade"):
            with self.subTest(model=model):
                self.assertEqual(self.imu_rate(self.render(model)), 1000.0)

    def test_both_models_accept_five_hundred_hertz(self) -> None:
        for model in ("crazyflie", "crazyflie_thrust_upgrade"):
            with self.subTest(model=model):
                self.assertEqual(
                    self.imu_rate(self.render(model, "--imu-rate", "500")), 500.0
                )

    def test_both_models_generate_four_eight_by_eight_nano_tof_sensors(self) -> None:
        for model in ("crazyflie", "crazyflie_thrust_upgrade"):
            with self.subTest(model=model):
                root = ET.fromstring(self.render(model, "--cf_id", "3"))
                sensors = root.findall(".//sensor[@type='gpu_lidar']")
                self.assertEqual(len(sensors), 4)
                for sensor_id, sensor in enumerate(sensors):
                    self.assertEqual(sensor.get("name"), f"nano_tof_{sensor_id}")
                    topic = sensor.findtext("topic")
                    self.assertEqual(topic, f"/cf_3/tof/{sensor_id}")
                    self.assertEqual(
                        sensor.findtext("ray/scan/horizontal/samples"), "8"
                    )
                    self.assertEqual(
                        sensor.findtext("ray/scan/vertical/samples"), "8"
                    )
                    self.assertEqual(sensor.findtext("update_rate"), "15")
                    pose = sensor.findtext("pose")
                    self.assertIsNotNone(pose)
                    assert pose is not None
                    self.assertEqual(len(pose.split()), 6)

    def test_rejects_rates_not_supported_by_the_sitl_firmware(self) -> None:
        with tempfile.TemporaryDirectory():
            template = GAZEBO_DIR / "models" / "crazyflie" / "model.sdf.jinja"
            result = subprocess.run(
                [
                    "uv",
                    "run",
                    "--with",
                    "jinja2",
                    "--with",
                    "numpy",
                    "--",
                    "python",
                    str(GENERATOR),
                    str(template),
                    str(GAZEBO_DIR),
                    "--stdout",
                    "--imu-rate",
                    "250",
                ],
                check=False,
                cwd=GAZEBO_DIR,
                capture_output=True,
                text=True,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid choice", result.stderr)

    def test_launcher_uses_compiled_firmware_rate_from_cmake_cache(self) -> None:
        resolver = GAZEBO_DIR / "launch" / "imu_rate.sh"
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_path = Path(temporary_directory)
            (build_path / "CMakeCache.txt").write_text(
                "CRAZYSIM_RATE_MAIN_LOOP:STRING=500\n"
            )
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; resolve_sitl_imu_rate "$2"',
                    "test",
                    str(resolver),
                    str(build_path),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
        self.assertEqual(result.stdout.strip(), "500")

    def test_launcher_environment_override_takes_precedence(self) -> None:
        resolver = GAZEBO_DIR / "launch" / "imu_rate.sh"
        with tempfile.TemporaryDirectory() as temporary_directory:
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; resolve_sitl_imu_rate "$2"',
                    "test",
                    str(resolver),
                    temporary_directory,
                ],
                check=True,
                capture_output=True,
                text=True,
                env={**os.environ, "NCS_CRAZYSIM_IMU_RATE": "500"},
            )
        self.assertEqual(result.stdout.strip(), "500")

    def test_launcher_defaults_to_one_kilohertz_without_build_cache(self) -> None:
        resolver = GAZEBO_DIR / "launch" / "imu_rate.sh"
        clean_env = {
            key: value
            for key, value in os.environ.items()
            if key != "NCS_CRAZYSIM_IMU_RATE"
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; resolve_sitl_imu_rate "$2"',
                    "test",
                    str(resolver),
                    temporary_directory,
                ],
                check=True,
                capture_output=True,
                text=True,
                env=clean_env,
            )
        self.assertEqual(result.stdout.strip(), "1000")


if __name__ == "__main__":
    unittest.main()
