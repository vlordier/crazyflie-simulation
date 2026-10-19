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

    def motor_model_parameters(self, sdf: str) -> tuple[float, float, float, float]:
        root = ET.fromstring(sdf)
        rotor_plugins = root.findall(
            ".//plugin[@name='gz::sim::systems::MulticopterMotorModel']"
        )
        interface = root.find(
            ".//plugin[@name='crazyflie_interface::GzCrazyflieInterface']"
        )
        self.assertEqual(len(rotor_plugins), 4)
        self.assertIsNotNone(interface)
        assert interface is not None
        rotor_parameters = {
            (
                float(plugin.findtext("motorConstant", "nan")),
                float(plugin.findtext("maxRotVelocity", "nan")),
            )
            for plugin in rotor_plugins
        }
        self.assertEqual(len(rotor_parameters), 1)
        return (
            float(interface.findtext("motorConstant", "nan")),
            float(interface.findtext("maxRotVelocity", "nan")),
            float(interface.findtext("pwmToThrustA", "nan")),
            float(interface.findtext("pwmToThrustB", "nan")),
        )

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

    def test_pwm_converter_uses_the_selected_gazebo_motor_model(self) -> None:
        expected_parameters = {
            "crazyflie": (1.7965e-8, 3052.0, 0.091492681, 0.067673604),
            "crazyflie_thrust_upgrade": (
                2.3375e-8,
                2797.0,
                0.091492681,
                0.067673604,
            ),
        }
        for model, expected in expected_parameters.items():
            with self.subTest(model=model):
                sdf = self.render(model)
                root = ET.fromstring(sdf)
                rotor = root.find(
                    ".//plugin[@name='gz::sim::systems::MulticopterMotorModel']"
                )
                self.assertIsNotNone(rotor)
                assert rotor is not None
                self.assertEqual(
                    self.motor_model_parameters(sdf),
                    (
                        float(rotor.findtext("motorConstant", "nan")),
                        float(rotor.findtext("maxRotVelocity", "nan")),
                        *expected[2:],
                    ),
                )
                self.assertEqual(self.motor_model_parameters(sdf), expected)

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
