import hid
import time
import json
import os
import threading
import rclpy
from control_drive.robot_controller import RobotController


# Where to save the controller mapping so calibration only happens once
MAPPING_FILE = os.path.expanduser("~/.ros/gamepad_mapping.json")

# Dead zone — axis values this close to center are treated as zero
DEAD_ZONE = 15


# ------------------------------------------------------------------
# GAMEPAD DETECTOR
# ------------------------------------------------------------------

def find_gamepad():
    """
    Scan all HID devices and return the first gamepad or joystick found.

    HID usage page 1 = Generic Desktop Controls
    Usage 4 = Joystick
    Usage 5 = Gamepad

    Returns:
        dict with device info, or None if no gamepad found
    """
    devices = hid.enumerate()
    for device in devices:
        if device["usage_page"] == 1 and device["usage"] in [4, 5]:
            return device
    return None


# ------------------------------------------------------------------
# AXIS CALIBRATOR
# ------------------------------------------------------------------

class AxisCalibrator:
    """
    Determines axis mapping by recording which bytes change
    when the user moves each stick.

    Uses noise filtering to ignore bytes that change constantly
    (like timestamps or packet counters) even when sticks are idle.
    """

    def __init__(self, device: hid.Device):
        self.device = device

    def _countdown(self, seconds: int):
        """Print a countdown so the user has time to get ready."""
        for i in range(seconds, 0, -1):
            print(f"\r  Starting in {i}...  ", end="", flush=True)
            time.sleep(1)
        print("\r  GO! Move the stick now!      ", flush=True)

    def _read_samples(self, duration: float) -> list:
        """Read HID reports for `duration` seconds and return all samples."""
        samples = []
        start = time.time()
        while time.time() - start < duration:
            data = self.device.read(64)
            if data:
                samples.append(list(data))
            time.sleep(0.02)
        return samples

    def _get_noisy_bytes(self, baseline_samples: list) -> set:
        """
        Find bytes that change even when sticks are at rest.
        These are counters, timestamps, or noise — we must ignore them.
        """
        noisy = set()
        if len(baseline_samples) < 2:
            return noisy
        first = baseline_samples[0]
        for sample in baseline_samples[1:]:
            for i, (a, b) in enumerate(zip(first, sample)):
                if abs(a - b) > 5:
                    noisy.add(i)
        return noisy

    def _find_axis(self, baseline: list, noisy_bytes: set, duration: float) -> int:
        """
        Record samples while user moves a stick.
        Returns the byte index that changed the most vs baseline,
        ignoring known noisy bytes.
        """
        samples = self._read_samples(duration)
        if not samples:
            return -1

        # Sum total change per byte across all samples
        total_change = {}
        for sample in samples:
            for i, (val, base) in enumerate(zip(sample, baseline)):
                if i not in noisy_bytes:
                    total_change[i] = total_change.get(i, 0) + abs(val - base)

        if not total_change:
            return -1

        best_index = max(total_change, key=total_change.get)
        best_change = total_change[best_index]

        # Must exceed threshold to count as a real axis
        return best_index if best_change > 50 else -1

    def calibrate(self) -> dict:
        """
        Guide the user through moving each stick to determine axis mapping.

        Returns:
            dict with keys: linear_axis, angular_axis,
                            linear_inverted, angular_inverted
        """
        print("\n=== Gamepad Calibration ===")
        print("This only runs once. Mapping will be saved for future use.")
        print("Follow each step carefully — you have 4 seconds to get ready.\n")

        # Step 1 — Baseline: keep sticks centered
        print("Step 1/4: Keep ALL sticks centered. Don't touch the controller.")
        self._countdown(4)
        baseline_samples = self._read_samples(3.0)
        if not baseline_samples:
            raise RuntimeError("No data from controller during baseline")

        baseline = [
            round(sum(s[i] for s in baseline_samples) / len(baseline_samples))
            for i in range(len(baseline_samples[0]))
        ]
        noisy_bytes = self._get_noisy_bytes(baseline_samples)
        print(f"  Baseline captured. Ignoring {len(noisy_bytes)} noisy bytes.")

        # Step 2 — Linear axis
        print("\nStep 2/4: Push the FORWARD/BACKWARD stick fully UP and hold it.")
        self._countdown(4)
        linear_axis = self._find_axis(baseline, noisy_bytes, duration=4.0)
        print("  Release the stick.")
        if linear_axis == -1:
            print("  No axis detected — using default byte 2")
            linear_axis = 2
        else:
            print(f"  Linear axis detected at byte {linear_axis}")
        time.sleep(2.0)

        # Step 3 — Angular axis (exclude linear byte from search)
        print("\nStep 3/4: Push the TURNING stick fully RIGHT and hold it.")
        self._countdown(4)
        noisy_for_angular = noisy_bytes | {linear_axis}
        angular_axis = self._find_axis(baseline, noisy_for_angular, duration=4.0)
        print("  Release the stick.")
        if angular_axis == -1:
            print("  No axis detected — using default byte 3")
            angular_axis = 3
        else:
            print(f"  Angular axis detected at byte {angular_axis}")
        time.sleep(2.0)

        # Step 4 — Check linear inversion
        print("\nStep 4/4: Push the FORWARD stick UP one more time and hold it.")
        self._countdown(4)
        samples = self._read_samples(3.0)
        print("  Release the stick.")

        linear_inverted = False
        if samples:
            avg = sum(s[linear_axis] for s in samples) / len(samples)
            # Pushing up = value less than center = needs inversion
            linear_inverted = avg < baseline[linear_axis]

        mapping = {
            "linear_axis":     linear_axis,
            "angular_axis":    angular_axis,
            "linear_inverted": linear_inverted,
            "angular_inverted": True,
            "center":          baseline[linear_axis],
        }

        print(f"\n=== Calibration Complete! ===")
        print(f"  Linear  axis: byte {linear_axis} (inverted: {linear_inverted})")
        print(f"  Angular axis: byte {angular_axis}")
        print(f"  To recalibrate: rm ~/.ros/gamepad_mapping.json\n")

        return mapping


# ------------------------------------------------------------------
# GAMEPAD CONTROLLER NODE
# ------------------------------------------------------------------

class GamepadController(RobotController):
    """
    Generic gamepad/joystick input source.

    Works with any HID gamepad — PS4, PS5, Xbox, or generic controllers.
    Auto-detects the connected device and loads or creates an axis mapping.

    CONTROL SCHEME (configured during calibration):
        Forward/backward stick → linear speed
        Turning stick          → angular speed

    TRIGGERS (bytes 8 and 9 — works for PS4, PS5, many others):
        R2/RT → increase speed smoothly (up to 2x at full press)
        L2/LT → decrease speed smoothly (down to 0.3x at full press)
        Both  → emergency stop

    OPTIONS/START button → quit
    """

    def __init__(self):
        super().__init__(
            node_name="gamepad_controller",
            default_speed=1.0,
            default_turn=1.0,
        )

        # Step 1 — Find a gamepad
        device_info = find_gamepad()
        if device_info is None:
            self.get_logger().error(
                "No gamepad detected! Make sure your controller is connected."
            )
            raise RuntimeError("No gamepad found")

        self.get_logger().info(
            f"Found: {device_info['manufacturer_string']} "
            f"{device_info['product_string']} "
            f"(VID: {hex(device_info['vendor_id'])} "
            f"PID: {hex(device_info['product_id'])})"
        )

        # Step 2 — Open the device
        self.device = hid.Device(
            device_info["vendor_id"],
            device_info["product_id"]
        )
        self.device.nonblocking = True

        # Step 3 — Load or calibrate axis mapping
        self.mapping = self._load_or_calibrate()

        self.running = True

        # State tracking for status display
        self.current_linear     = 0.0
        self.current_angular    = 0.0
        self.current_multiplier = 1.0
        self.current_mode       = "IDLE"

        self.create_timer(1.0, self._print_status)

        self.get_logger().info("Gamepad controller ready.")
        self.get_logger().info(
            "R2=faster  L2=slower  L2+R2=STOP  OPTIONS/START=quit"
        )
        self.get_logger().info(
            "To recalibrate: rm ~/.ros/gamepad_mapping.json and restart."
        )

    # ------------------------------------------------------------------
    # MAPPING PERSISTENCE
    # ------------------------------------------------------------------

    def _load_or_calibrate(self) -> dict:
        """Load saved mapping or run calibration if none exists."""
        if os.path.exists(MAPPING_FILE):
            with open(MAPPING_FILE, "r") as f:
                mapping = json.load(f)
            self.get_logger().info(f"Loaded saved mapping from {MAPPING_FILE}")
            self.get_logger().info(
                f"Linear axis: byte {mapping['linear_axis']}  "
                f"Angular axis: byte {mapping['angular_axis']}"
            )
            return mapping

        # No saved mapping — run calibration
        self.get_logger().info("No saved mapping found. Starting calibration...")
        calibrator = AxisCalibrator(self.device)
        mapping = calibrator.calibrate()

        # Save for next time
        os.makedirs(os.path.dirname(MAPPING_FILE), exist_ok=True)
        with open(MAPPING_FILE, "w") as f:
            json.dump(mapping, f, indent=2)
        self.get_logger().info(f"Mapping saved to {MAPPING_FILE}")

        return mapping

    # ------------------------------------------------------------------
    # STATUS DISPLAY
    # ------------------------------------------------------------------

    def _print_status(self):
        """Print current controller state to terminal every second."""
        print(
            f"\r[GAMEPAD] "
            f"Mode: {self.current_mode:<12} | "
            f"Linear: {self.current_linear:+.2f} m/s | "
            f"Angular: {self.current_angular:+.2f} rad/s | "
            f"Speed: {self.current_multiplier:.2f}x",
            end="",
            flush=True,
        )

    # ------------------------------------------------------------------
    # AXIS NORMALIZATION
    # ------------------------------------------------------------------

    def _normalize(self, raw: int, inverted: bool = False) -> float:
        """Convert raw axis value (0-255) to normalized float (-1.0 to 1.0)."""
        centered = raw - 128
        if abs(centered) < DEAD_ZONE:
            return 0.0
        if centered > 0:
            value = (centered - DEAD_ZONE) / (127 - DEAD_ZONE)
        else:
            value = (centered + DEAD_ZONE) / (127 - DEAD_ZONE)
        return -value if inverted else value

    # ------------------------------------------------------------------
    # REPORT PARSING
    # ------------------------------------------------------------------

    def _parse_report(self, data: list):
        """
        Parse HID report using the saved axis mapping.

        Axis bytes are determined during calibration and saved to:
            ~/.ros/gamepad_mapping.json

        Trigger bytes (8=L2, 9=R2) work for PS4/PS5/many controllers.
        If your controller lacks analog triggers these will stay at 0.
        """
        lin_axis = self.mapping["linear_axis"]
        ang_axis = self.mapping["angular_axis"]
        lin_inv  = self.mapping["linear_inverted"]
        ang_inv  = self.mapping["angular_inverted"]

        linear  = self._normalize(data[lin_axis], inverted=lin_inv) * self.speed
        angular = self._normalize(data[ang_axis], inverted=ang_inv) * self.turn

        # Triggers
        l2 = data[8] if len(data) > 8 else 0
        r2 = data[9] if len(data) > 9 else 0

        emergency_stop = l2 > 200 and r2 > 200

        multiplier = 1.0
        if not emergency_stop:
            if r2 > 10:
                multiplier = 1.0 + (r2 / 255.0)
            elif l2 > 10:
                multiplier = 1.0 - (l2 / 255.0 * 0.7)

        linear  *= multiplier
        angular *= multiplier

        # OPTIONS/START button
        quit_pressed = bool(data[5] & 0x10) if len(data) > 5 else False

        # Mode for status display
        if emergency_stop:
            mode = "EMERGENCY"
        elif abs(linear) > 0.01 and abs(angular) > 0.01:
            mode = "CURVE"
        elif abs(linear) > 0.01:
            mode = "FORWARD" if linear > 0 else "BACKWARD"
        elif abs(angular) > 0.01:
            mode = "TURNING"
        else:
            mode = "IDLE"

        return linear, angular, multiplier, emergency_stop, quit_pressed, mode

    # ------------------------------------------------------------------
    # MAIN INPUT LOOP
    # ------------------------------------------------------------------

    def run(self):
        """Read gamepad HID data in a loop and translate to robot commands."""
        self.get_logger().info("Gamepad input loop started.")
        last_data_time = time.time()

        try:
            while self.running and rclpy.ok():
                data = self.device.read(64)

                if not data:
                    if time.time() - last_data_time > 0.5:
                        self.stop()
                        self.current_mode = "IDLE"
                    time.sleep(0.01)
                    continue

                last_data_time = time.time()

                linear, angular, multiplier, emergency_stop, quit_pressed, mode = \
                    self._parse_report(data)

                self.current_linear     = linear
                self.current_angular    = angular
                self.current_multiplier = multiplier
                self.current_mode       = mode

                if quit_pressed:
                    print()
                    self.get_logger().info("Quit button pressed — stopping.")
                    self.running = False
                    break

                if emergency_stop:
                    print()
                    self.get_logger().warn("EMERGENCY STOP!")
                    self.stop()
                    continue

                self.send_command(linear=linear, angular=angular)

        except Exception as e:
            self.get_logger().error(f"Gamepad error: {e}")

        finally:
            print()
            self.stop()
            self.device.close()
            self.get_logger().info("Gamepad disconnected.")


def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = GamepadController()

        input_thread = threading.Thread(
            target=controller.run,
            daemon=True
        )
        input_thread.start()

        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if controller:
            controller.stop()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()