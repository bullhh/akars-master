//! Persistent arm-pose calibration for the SG2002 ball-picking robot.

use std::fmt;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;
use std::str::FromStr;

pub const ARM_ANGLE_MAX: f32 = 270.0;
pub const DEFAULT_STOP_AREA: f32 = 0.50;
pub const STOP_AREA_MIN: f32 = 0.05;
pub const STOP_AREA_MAX: f32 = 0.90;
pub const DEFAULT_CENTER_OFFSET_PX: i32 = 75;
pub const CENTER_OFFSET_PX_MIN: i32 = -240;
pub const CENTER_OFFSET_PX_MAX: i32 = 240;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmPose {
    pub servo0: f32,
    pub servo1: f32,
    pub servo2: f32,
}

impl ArmPose {
    pub fn new(servo0: f32, servo1: f32, servo2: f32) -> io::Result<Self> {
        let pose = Self {
            servo0,
            servo1,
            servo2,
        };
        pose.validate()?;
        Ok(pose)
    }

    fn parse(value: &str, line_number: usize) -> io::Result<Self> {
        let values = value
            .split(',')
            .map(str::trim)
            .map(|angle| {
                angle.parse::<f32>().map_err(|_| {
                    invalid_data(format!("line {line_number}: invalid arm angle `{angle}`"))
                })
            })
            .collect::<io::Result<Vec<_>>>()?;
        if values.len() != 3 {
            return Err(invalid_data(format!(
                "line {line_number}: an arm pose requires exactly three angles"
            )));
        }
        Self::new(values[0], values[1], values[2])
            .map_err(|err| invalid_data(format!("line {line_number}: invalid arm pose: {err}")))
    }

    fn validate(self) -> io::Result<()> {
        for (servo, angle) in [self.servo0, self.servo1, self.servo2]
            .into_iter()
            .enumerate()
        {
            if !angle.is_finite() || !(0.0..=ARM_ANGLE_MAX).contains(&angle) {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "servo{servo} angle must be finite and within 0..={ARM_ANGLE_MAX}, got {angle}"
                    ),
                ));
            }
        }
        Ok(())
    }
}

impl fmt::Display for ArmPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{:.1},{:.1},{:.1}",
            self.servo0, self.servo1, self.servo2
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArmPoseName {
    Ready,
    Approach,
    Closed,
    Lift,
}

impl ArmPoseName {
    pub const ALL: [Self; 4] = [Self::Ready, Self::Approach, Self::Closed, Self::Lift];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Ready => "ready",
            Self::Approach => "approach",
            Self::Closed => "closed",
            Self::Lift => "lift",
        }
    }
}

impl fmt::Display for ArmPoseName {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl FromStr for ArmPoseName {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.trim() {
            "ready" => Ok(Self::Ready),
            "approach" | "grab" => Ok(Self::Approach),
            "closed" | "close" => Ok(Self::Closed),
            "lift" | "show" => Ok(Self::Lift),
            other => Err(format!(
                "unknown arm pose `{other}`; expected ready, approach, closed, or lift"
            )),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmCalibration {
    pub ready: ArmPose,
    pub approach: ArmPose,
    pub closed: ArmPose,
    pub lift: ArmPose,
    pub stop_area: f32,
    pub center_offset_px: i32,
}

impl Default for ArmCalibration {
    fn default() -> Self {
        Self {
            ready: ArmPose {
                servo0: 150.0,
                servo1: 100.0,
                servo2: 140.0,
            },
            approach: ArmPose {
                servo0: 245.0,
                servo1: 110.0,
                servo2: 150.0,
            },
            closed: ArmPose {
                servo0: 245.0,
                servo1: 110.0,
                servo2: 80.0,
            },
            lift: ArmPose {
                servo0: 150.0,
                servo1: 100.0,
                servo2: 60.0,
            },
            stop_area: DEFAULT_STOP_AREA,
            center_offset_px: DEFAULT_CENTER_OFFSET_PX,
        }
    }
}

impl ArmCalibration {
    pub fn load(path: &Path) -> io::Result<Self> {
        let content = fs::read_to_string(path)?;
        Self::parse(&content)
    }

    pub fn parse(content: &str) -> io::Result<Self> {
        let mut calibration = Self::default();
        for (index, raw_line) in content.lines().enumerate() {
            let line_number = index + 1;
            let line = raw_line.split('#').next().unwrap_or_default().trim();
            if line.is_empty() {
                continue;
            }
            let (name, value) = line.split_once('=').ok_or_else(|| {
                invalid_data(format!("line {line_number}: expected `name=a,b,c`"))
            })?;
            let name = name.trim();
            if name == "stop_area" {
                let stop_area = value.trim().parse::<f32>().map_err(|_| {
                    invalid_data(format!("line {line_number}: invalid stop_area `{value}`"))
                })?;
                calibration.set_stop_area(stop_area).map_err(|err| {
                    invalid_data(format!("line {line_number}: invalid stop_area: {err}"))
                })?;
                continue;
            }
            if name == "center_offset_px" {
                let center_offset_px = value.trim().parse::<i32>().map_err(|_| {
                    invalid_data(format!(
                        "line {line_number}: invalid center_offset_px `{value}`"
                    ))
                })?;
                calibration
                    .set_center_offset_px(center_offset_px)
                    .map_err(|err| {
                        invalid_data(format!(
                            "line {line_number}: invalid center_offset_px: {err}"
                        ))
                    })?;
                continue;
            }
            let name = name.parse::<ArmPoseName>().map_err(|err| {
                invalid_data(format!(
                    "line {line_number}: {err}; or expected stop_area/center_offset_px"
                ))
            })?;
            let pose = ArmPose::parse(value, line_number)?;
            calibration.set(name, pose);
        }
        Ok(calibration)
    }

    pub fn pose(self, name: ArmPoseName) -> ArmPose {
        match name {
            ArmPoseName::Ready => self.ready,
            ArmPoseName::Approach => self.approach,
            ArmPoseName::Closed => self.closed,
            ArmPoseName::Lift => self.lift,
        }
    }

    pub fn set(&mut self, name: ArmPoseName, pose: ArmPose) {
        match name {
            ArmPoseName::Ready => self.ready = pose,
            ArmPoseName::Approach => self.approach = pose,
            ArmPoseName::Closed => self.closed = pose,
            ArmPoseName::Lift => self.lift = pose,
        }
    }

    pub fn set_stop_area(&mut self, stop_area: f32) -> io::Result<()> {
        if !stop_area.is_finite() || !(STOP_AREA_MIN..=STOP_AREA_MAX).contains(&stop_area) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "stop_area must be finite and within {STOP_AREA_MIN:.2}..={STOP_AREA_MAX:.2}, got {stop_area}"
                ),
            ));
        }
        self.stop_area = stop_area;
        Ok(())
    }

    pub fn set_center_offset_px(&mut self, center_offset_px: i32) -> io::Result<()> {
        if !(CENTER_OFFSET_PX_MIN..=CENTER_OFFSET_PX_MAX).contains(&center_offset_px) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "center_offset_px must be within {CENTER_OFFSET_PX_MIN}..={CENTER_OFFSET_PX_MAX}, got {center_offset_px}"
                ),
            ));
        }
        self.center_offset_px = center_offset_px;
        Ok(())
    }

    pub fn save(self, path: &Path) -> io::Result<()> {
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
        let temporary = path.with_extension("tmp");
        let mut file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open(&temporary)?;
        write!(file, "{self}")?;
        file.sync_all()?;
        fs::rename(temporary, path)
    }
}

impl fmt::Display for ArmCalibration {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(formatter, "# AKARS arm calibration: servo0,servo1,servo2")?;
        for name in ArmPoseName::ALL {
            writeln!(formatter, "{name}={}", self.pose(name))?;
        }
        writeln!(
            formatter,
            "# Larger stop_area means the car stops closer to the ball"
        )?;
        writeln!(formatter, "stop_area={:.3}", self.stop_area)?;
        writeln!(
            formatter,
            "# Positive center_offset_px places the ball to the image right"
        )?;
        writeln!(formatter, "center_offset_px={}", self.center_offset_px)?;
        Ok(())
    }
}

fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

#[cfg(test)]
mod tests {
    use super::{ArmCalibration, ArmPose, ArmPoseName};

    #[test]
    fn uses_board_verified_defaults() {
        let calibration = ArmCalibration::default();

        assert_eq!(calibration.stop_area, 0.50);
        assert_eq!(calibration.center_offset_px, 75);
        assert_eq!(
            calibration.approach,
            ArmPose::new(245.0, 110.0, 150.0).unwrap()
        );
        assert_eq!(
            calibration.closed,
            ArmPose::new(245.0, 110.0, 80.0).unwrap()
        );
    }

    #[test]
    fn parses_partial_calibration_over_defaults() {
        let calibration = ArmCalibration::parse(
            "# tuned on the board\nready=181,186,141\nclosed=244.5,184.5,88\n",
        )
        .unwrap();

        assert_eq!(
            calibration.ready,
            ArmPose::new(181.0, 186.0, 141.0).unwrap()
        );
        assert_eq!(calibration.approach, ArmCalibration::default().approach);
        assert_eq!(calibration.closed.servo2, 88.0);
    }

    #[test]
    fn formats_a_round_trip_configuration() {
        let expected = ArmCalibration::default();
        let parsed = ArmCalibration::parse(&expected.to_string()).unwrap();

        assert_eq!(parsed, expected);
        assert_eq!(parsed.pose(ArmPoseName::Lift), expected.lift);
    }

    #[test]
    fn rejects_out_of_range_angles() {
        let err = ArmCalibration::parse("ready=180,185,271\n").unwrap_err();

        assert!(err.to_string().contains("0..=270"));
    }

    #[test]
    fn rejects_unknown_pose_names() {
        let err = ArmCalibration::parse("unknown=180,185,140\n").unwrap_err();

        assert!(err.to_string().contains("unknown arm pose"));
    }

    #[test]
    fn parses_and_validates_stop_area() {
        let calibration = ArmCalibration::parse("stop_area=0.50\n").unwrap();
        assert_eq!(calibration.stop_area, 0.50);

        let err = ArmCalibration::parse("stop_area=0.95\n").unwrap_err();
        assert!(err.to_string().contains("0.05..=0.90"));
    }

    #[test]
    fn parses_and_validates_center_offset() {
        let calibration = ArmCalibration::parse("center_offset_px=70\n").unwrap();
        assert_eq!(calibration.center_offset_px, 70);

        let err = ArmCalibration::parse("center_offset_px=300\n").unwrap_err();
        assert!(err.to_string().contains("-240..=240"));
    }
}
