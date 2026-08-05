use crate::arm_config::{ArmCalibration, ArmPose, ARM_ANGLE_MAX};
use crate::serial::SerialPort;
use std::io;
use std::thread;
use std::time::Duration;

const PULSE_MIN: i32 = 500;
const PULSE_MAX: i32 = 2500;

pub struct Arm {
    port: SerialPort,
    calibration: ArmCalibration,
}

impl Arm {
    pub fn open(device: &str, baudrate: i32) -> io::Result<Self> {
        Self::open_with_calibration(device, baudrate, ArmCalibration::default())
    }

    pub fn open_with_calibration(
        device: &str,
        baudrate: i32,
        calibration: ArmCalibration,
    ) -> io::Result<Self> {
        crate::pinmux::configure_for_device(device);
        Ok(Self {
            port: SerialPort::open(device, baudrate, false)?,
            calibration,
        })
    }

    pub fn set_angle(&mut self, servo_id: i32, angle: f32, time_ms: i32) {
        let clamped = angle.clamp(0.0, ARM_ANGLE_MAX);
        let pulse = angle_to_pulse(clamped);
        self.send_command(&format!("#{servo_id:03}P{pulse:04}T{time_ms}!"));
    }

    pub fn move_to_pose(&mut self, pose: ArmPose, time_ms: i32) {
        self.set_angle(0, pose.servo0, time_ms);
        self.set_angle(1, pose.servo1, time_ms);
        self.set_angle(2, pose.servo2, time_ms);
    }

    pub fn release_torque(&mut self, servo_id: i32) {
        self.send_command(&format!("#{servo_id:03}PULK"));
    }

    pub fn restore_torque(&mut self, servo_id: i32) {
        self.send_command(&format!("#{servo_id:03}PULR"));
    }

    pub fn grab(&mut self) {
        self.move_to_pose(self.calibration.approach, 1000);
        sleep_ms(1500);

        self.move_to_pose(self.calibration.closed, 1000);
        sleep_ms(1000);

        self.move_to_pose(self.calibration.lift, 1000);
        sleep_ms(1200);
    }

    pub fn release_pos(&mut self) {
        self.move_to_pose(self.calibration.ready, 1000);
    }

    pub fn release(&mut self) {
        self.set_angle(2, self.calibration.ready.servo2, 1000);
    }

    pub fn grab_pos(&mut self) {
        self.move_to_pose(self.calibration.ready, 1000);
    }

    pub fn show(&mut self) {
        self.move_to_pose(self.calibration.lift, 1000);
    }

    fn send_command(&mut self, command: &str) {
        if let Err(err) = self.port.write_all_drain(command.as_bytes()) {
            eprintln!("[arm] write failed: {err}");
        }
    }
}

fn angle_to_pulse(angle: f32) -> i32 {
    ((500.0 + (angle / ARM_ANGLE_MAX) * 2000.0).round() as i32).clamp(PULSE_MIN, PULSE_MAX)
}

fn sleep_ms(ms: u64) {
    thread::sleep(Duration::from_millis(ms));
}

#[cfg(test)]
mod tests {
    use super::angle_to_pulse;

    #[test]
    fn converts_angles_to_pulses() {
        assert_eq!(angle_to_pulse(0.0), 500);
        assert_eq!(angle_to_pulse(270.0), 2500);
        assert_eq!(angle_to_pulse(135.0), 1500);
    }
}
