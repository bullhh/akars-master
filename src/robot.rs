use crate::arm::Arm;
use crate::arm_config::{DEFAULT_CENTER_OFFSET_PX, DEFAULT_STOP_AREA};
use crate::camera::UsbCamera;
use crate::detector::Detection;
use crate::image_bridge;
use crate::motor::Motor;
use crate::tpu::{InferTiming, InferenceConfig, ModelInputFormat, YoloModel};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

static STOP_REQUESTED: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Debug)]
pub struct RobotConfig {
    pub inference: InferenceConfig,
    pub max_frames: Option<u64>,
    pub stop_area: f32,
    pub center_offset_px: i32,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            inference: InferenceConfig::default(),
            max_frames: None,
            stop_area: DEFAULT_STOP_AREA,
            center_offset_px: DEFAULT_CENTER_OFFSET_PX,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum RobotStatus {
    ChaseTennis,
    GrabTennis,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ApproachPhase {
    Fast,
    Medium,
    Fine,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SteeringState {
    Straight,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug)]
struct RobotState {
    status: RobotStatus,
    area_ratio: f32,
    ball_cx: i32,
    grab_confirm_count: i32,
    // 连续漏检帧数;在宽限期内先保持不动等球重现,超过阈值才判定真丢并搜索。
    miss_count: i32,
    area_samples: [f32; 3],
    area_sample_count: usize,
    area_sample_next: usize,
    previous_raw_area: Option<f32>,
    last_motion_was_forward: bool,
    pending_alignment_error: Option<i32>,
    ineffective_alignment_count: i32,
    approach_phase: ApproachPhase,
    positioning: bool,
    filtered_ball_cx: Option<f32>,
    steering: SteeringState,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            status: RobotStatus::ChaseTennis,
            area_ratio: 0.0,
            ball_cx: 0,
            grab_confirm_count: 0,
            miss_count: 0,
            area_samples: [0.0; 3],
            area_sample_count: 0,
            area_sample_next: 0,
            previous_raw_area: None,
            last_motion_was_forward: false,
            pending_alignment_error: None,
            ineffective_alignment_count: 0,
            approach_phase: ApproachPhase::Fast,
            positioning: false,
            filtered_ball_cx: None,
            steering: SteeringState::Straight,
        }
    }
}

impl RobotState {
    fn filtered_area(&mut self, area: f32) -> f32 {
        self.area_samples[self.area_sample_next] = area;
        self.area_sample_next = (self.area_sample_next + 1) % self.area_samples.len();
        self.area_sample_count = (self.area_sample_count + 1).min(self.area_samples.len());

        let mut values = self.area_samples;
        values[..self.area_sample_count].sort_by(f32::total_cmp);
        match self.area_sample_count {
            1 => values[0],
            2 => (values[0] + values[1]) * 0.5,
            _ => values[1],
        }
    }

    fn predicted_area(&mut self, area: f32) -> f32 {
        let growth = if self.last_motion_was_forward {
            self.previous_raw_area
                .map(|previous| (area - previous).clamp(0.0, MAX_AREA_PREDICTION_STEP))
                .unwrap_or(0.0)
        } else {
            0.0
        };
        self.previous_raw_area = Some(area);
        self.last_motion_was_forward = false;
        (area + growth).min(1.0)
    }

    fn observe_alignment_result(&mut self, current_error: i32) {
        let Some(previous_error) = self.pending_alignment_error.take() else {
            return;
        };
        let improvement = previous_error - current_error.abs();
        if improvement < STUCK_PIXEL_THRESHOLD {
            self.ineffective_alignment_count += 1;
            eprintln!(
                "[control] alignment ineffective: before={} now={} improvement={} count={}/{}",
                previous_error,
                current_error.abs(),
                improvement,
                self.ineffective_alignment_count,
                STUCK_ACTION_THRESHOLD
            );
        } else {
            self.ineffective_alignment_count = 0;
        }
    }

    fn remember_alignment_action(&mut self, error: i32) {
        self.pending_alignment_error = Some(error.abs());
    }

    fn filtered_ball_x(&mut self, ball_cx: i32) -> i32 {
        let filtered = self
            .filtered_ball_cx
            .map(|previous| {
                previous * (1.0 - BALL_X_FILTER_ALPHA) + ball_cx as f32 * BALL_X_FILTER_ALPHA
            })
            .unwrap_or(ball_cx as f32);
        self.filtered_ball_cx = Some(filtered);
        filtered.round() as i32
    }

    fn update_steering(&mut self, error: i32, image_w: i32) -> SteeringState {
        let enter = scaled_reference_px(STEER_ENTER_PX, image_w);
        let exit = scaled_reference_px(STEER_EXIT_PX, image_w);
        self.steering = match self.steering {
            SteeringState::Straight if error < -enter => SteeringState::Left,
            SteeringState::Straight if error > enter => SteeringState::Right,
            SteeringState::Left if error >= -exit => SteeringState::Straight,
            SteeringState::Right if error <= exit => SteeringState::Straight,
            state => state,
        };
        self.steering
    }

    fn reset_approach(&mut self) {
        self.approach_phase = ApproachPhase::Fast;
        self.positioning = false;
        self.filtered_ball_cx = None;
        self.steering = SteeringState::Straight;
        self.pending_alignment_error = None;
        self.ineffective_alignment_count = 0;
    }

    fn enter_approach_phase(&mut self, phase: ApproachPhase) {
        if self.approach_phase != phase {
            self.approach_phase = phase;
            self.steering = SteeringState::Straight;
            self.pending_alignment_error = None;
            self.ineffective_alignment_count = 0;
        }
    }
}

// 参考帧宽度。像素类阈值(如中心死区)以此为基准按分辨率缩放,保证不同分辨率下行为一致。
const REFERENCE_FRAME_WIDTH: f32 = 640.0;
const DISTANCE_MARGIN: f32 = 0.02;
// 速度分段按停车面积的比例计算，避免 stop_area 从 0.75 调到 0.50 后过早降速。
// stop_area=0.50 时，distance_low=0.48，高速/中速出口约为 0.34/0.43。
const FAST_EXIT_RATIO: f32 = 0.70;
const MEDIUM_EXIT_RATIO: f32 = 0.90;
// 进入最终定位后采用更宽的距离滞回；只有明显越界才重新前进或后退。
const POSITION_DISTANCE_MARGIN: f32 = 0.05;
const GRAB_CENTER_MARGIN_PX: i32 = 35;
const STEER_ENTER_PX: i32 = 60;
const STEER_EXIT_PX: i32 = 30;
const BALL_X_FILTER_ALPHA: f32 = 0.30;
// 连续满足“够近且居中”的帧数达到此值才真正执行抓取,防单帧误检导致误抓。
const GRAB_CONFIRM_THRESHOLD: i32 = 2;
// 丢失宽限帧数。看到过球后若漏检,先原地保持不动等它重现,连续漏检超过此值才判定真丢并开始搜索,
// 避免转向时的偶发漏检(运动模糊/擦边)立刻触发盲搜把球搞丢。
const LOST_GRACE_FRAMES: i32 = 8;

// 没检测到球时原地慢转搜索的速度。
const IDLE_SPEED: i32 = 18;
const FAR_DISTANCE_SPEED: i32 = 45;
const MEDIUM_DISTANCE_SPEED: i32 = 32;
const NEAR_DISTANCE_SPEED: i32 = 28;
const FAR_ARC_TURN_DELTA: i32 = 8;
const MEDIUM_ARC_TURN_DELTA: i32 = 6;
const NEAR_ARC_TURN_DELTA: i32 = 6;
const NEAR_MOVE_US: u64 = 70_000;
const POSITION_TURN_SPEED: i32 = 30;
const POSITION_TURN_MIN_US: u64 = 60_000;
const POSITION_TURN_MAX_US: u64 = 100_000;
const POSITION_SETTLE_US: u64 = 40_000;
const MAX_AREA_PREDICTION_STEP: f32 = 0.15;
const STRONG_TURN_SPEED: i32 = 30;
const STRONG_TURN_US: u64 = 150_000;
const STUCK_PIXEL_THRESHOLD: i32 = 8;
const STUCK_ACTION_THRESHOLD: i32 = 2;

pub fn install_signal_handlers() {
    unsafe {
        crate::linux::signal(crate::linux::SIGINT, signal_handler);
        crate::linux::signal(crate::linux::SIGTERM, signal_handler);
    }
}

extern "C" fn signal_handler(_: i32) {
    STOP_REQUESTED.store(true, Ordering::SeqCst);
}

pub fn stop_requested() -> bool {
    STOP_REQUESTED.load(Ordering::SeqCst)
}

pub fn run_tennis_hunter(
    mut camera: UsbCamera,
    mut model: YoloModel,
    mut motor: Motor,
    mut arm: Arm,
    config: RobotConfig,
) {
    let mut robot = RobotState::default();
    let mut total_time = Duration::ZERO;
    let mut frame_count = 0u64;
    let mut frame_idx = 0u64;
    let (model_w, model_h) = model.input_dimensions();
    let input_format = model.input_format();
    let input_len = input_format.buffer_len(model_w, model_h).unwrap_or(0);
    if input_len == 0 {
        eprintln!("[camera] invalid model input size: {}x{}", model_w, model_h);
        motor.standby();
        return;
    }
    eprintln!("[tpu] model input format: {input_format:?}");
    let mut input_buffer = vec![0u8; input_len];

    arm.grab_pos();

    while !stop_requested() {
        if let Some(limit) = config.max_frames {
            if frame_idx >= limit {
                break;
            }
        }

        frame_idx += 1;

        let capture_start = Instant::now();
        let frame = match camera.get_frame() {
            Ok(frame) => frame,
            Err(err) => {
                eprintln!("[camera] failed to get frame: {err}");
                sleep_us(100_000);
                continue;
            }
        };
        let capture_us = capture_start.elapsed().as_micros() as i64;

        let preprocess_start = Instant::now();
        let preprocess_result = match input_format {
            ModelInputFormat::RgbPlanar => image_bridge::yuv422p_to_rgb_planar(
                &frame.pixels,
                &mut input_buffer,
                model_w,
                model_h,
            ),
            ModelInputFormat::Yuv420Planar => {
                image_bridge::yuv422p_to_yuv420p(&frame.pixels, &mut input_buffer, model_w, model_h)
            }
            ModelInputFormat::YuvNv12 => {
                image_bridge::yuv422p_to_nv12(&frame.pixels, &mut input_buffer, model_w, model_h)
            }
            ModelInputFormat::YuvNv21 => {
                image_bridge::yuv422p_to_nv21(&frame.pixels, &mut input_buffer, model_w, model_h)
            }
        };
        if let Err(err) = preprocess_result {
            eprintln!("[pre] YUV preprocess failed: {err}");
            continue;
        }
        let preprocess_us = preprocess_start.elapsed().as_micros() as i64;

        let image_w = frame.width as i32;
        let image_h = frame.height as i32;
        let mut timing = InferTiming::default();
        let inference_result = match input_format {
            ModelInputFormat::RgbPlanar => model.infer_rgb_planar_timed(
                &input_buffer,
                image_w,
                image_h,
                config.inference,
                Some(&mut timing),
            ),
            ModelInputFormat::Yuv420Planar
            | ModelInputFormat::YuvNv12
            | ModelInputFormat::YuvNv21 => model.infer_yuv420_timed(
                &input_buffer,
                image_w,
                image_h,
                config.inference,
                Some(&mut timing),
            ),
        };
        let detections = match inference_result {
            Ok(detections) => detections,
            Err(err) => {
                eprintln!("[detect] inference failed: {err}");
                continue;
            }
        };

        eprintln!(
            "[time] frame={} capture={:.1} pre={:.1} fwd={:.1} post={:.1} ms",
            frame_idx,
            capture_us as f32 / 1000.0,
            preprocess_us as f32 / 1000.0,
            timing.forward_us as f32 / 1000.0,
            timing.postprocess_us as f32 / 1000.0,
        );

        let control_start = Instant::now();
        handle_detections(
            &detections,
            image_w,
            image_h,
            &mut robot,
            &mut motor,
            &mut arm,
            config.stop_area,
            config.center_offset_px,
        );

        let control_time = control_start.elapsed();
        let frame_time = Duration::from_micros(
            capture_us.max(0) as u64
                + preprocess_us.max(0) as u64
                + timing.forward_us.max(0) as u64
                + timing.postprocess_us.max(0) as u64,
        ) + control_time;
        total_time += frame_time;
        frame_count += 1;
        let fps = if frame_time.as_secs_f32() > 0.0 {
            1.0 / frame_time.as_secs_f32()
        } else {
            0.0
        };
        let avg_fps = if total_time.as_secs_f32() > 0.0 {
            frame_count as f32 / total_time.as_secs_f32()
        } else {
            0.0
        };
        println!(
            "[FPS] {:.2} avg: {:.2} ({:.1}ms, control={:.1}ms)",
            fps,
            avg_fps,
            frame_time.as_secs_f32() * 1000.0,
            control_time.as_secs_f32() * 1000.0
        );
    }

    motor.standby();
}

fn handle_detections(
    detections: &[Detection],
    image_w: i32,
    image_h: i32,
    robot: &mut RobotState,
    motor: &mut Motor,
    arm: &mut Arm,
    stop_area: f32,
    center_offset_px: i32,
) {
    if detections.is_empty() {
        robot.grab_confirm_count = 0;
        robot.pending_alignment_error = None;
        robot.area_sample_count = 0;
        robot.area_sample_next = 0;
        robot.previous_raw_area = None;
        robot.last_motion_was_forward = false;
        robot.miss_count += 1;

        if robot.miss_count <= LOST_GRACE_FRAMES {
            // 宽限期内:刚才还看得到球,先停住等它重现,别急着盲搜。
            eprintln!(
                "[detect] missed {}/{} frames, holding",
                robot.miss_count, LOST_GRACE_FRAMES
            );
            motor.standby();
        } else {
            // 真丢了:朝球最后出现的一侧转,而不是固定方向盲搜。
            robot.status = RobotStatus::ChaseTennis;
            let last_offset = robot.ball_cx - target_x(image_w, center_offset_px);
            if last_offset >= 0 {
                eprintln!("[detect] ball lost, searching right (last seen right)");
                motor.drive(IDLE_SPEED, -IDLE_SPEED);
            } else {
                eprintln!("[detect] ball lost, searching left (last seen left)");
                motor.drive(-IDLE_SPEED, IDLE_SPEED);
            }
            robot.reset_approach();
        }
        return;
    }
    robot.miss_count = 0;

    let best = detections
        .iter()
        .max_by(|a, b| {
            let area_a = a.bbox.w * a.bbox.h;
            let area_b = b.bbox.w * b.bbox.h;
            area_a.total_cmp(&area_b)
        })
        .expect("non-empty detections");

    let image_area = (image_w.max(1) * image_h.max(1)) as f32;
    let area_ratio = (best.bbox.w * best.bbox.h) / image_area;
    let filtered_area = robot.filtered_area(area_ratio);
    let predicted_area = robot.predicted_area(filtered_area);
    let raw_ball_cx = best.bbox.x as i32;
    let ball_cx = robot.filtered_ball_x(raw_ball_cx);
    let distance_low = (stop_area - DISTANCE_MARGIN).max(0.0);
    let position_low = (stop_area - POSITION_DISTANCE_MARGIN).max(0.0);
    let position_high = (stop_area + POSITION_DISTANCE_MARGIN).min(1.0);
    let (fast_exit, medium_exit) = approach_thresholds(distance_low);
    let target_phase = if robot.positioning {
        ApproachPhase::Fine
    } else {
        target_phase_for_area(
            robot.approach_phase,
            filtered_area,
            fast_exit,
            medium_exit,
            distance_low,
        )
    };
    let effective_center_offset = approach_center_offset(
        center_offset_px,
        target_phase,
        filtered_area,
        fast_exit,
        medium_exit,
    );
    let center = target_x(image_w, effective_center_offset);
    let center_margin = scaled_reference_px(GRAB_CENTER_MARGIN_PX, image_w);
    let offset = ball_cx - center;
    let centered = offset.abs() <= center_margin;

    if !robot.positioning {
        robot.observe_alignment_result(offset);
    }

    robot.area_ratio = area_ratio;
    robot.ball_cx = ball_cx;

    eprintln!(
        "[detect] phase={:?} positioning={} area={area_ratio:.3} filtered={filtered_area:.3} predicted={predicted_area:.3} cx_raw={raw_ball_cx} cx={ball_cx} target={center} error={offset} margin={center_margin} conf={:.3} centered={centered}",
        robot.approach_phase,
        robot.positioning,
        best.score,
    );

    if !robot.positioning && filtered_area >= distance_low {
        robot.positioning = true;
        robot.enter_approach_phase(ApproachPhase::Fine);
        robot.pending_alignment_error = None;
        robot.ineffective_alignment_count = 0;
        eprintln!(
            "[control] enter-positioning: filtered={filtered_area:.3} trigger={distance_low:.3} range=[{position_low:.3},{position_high:.3}]"
        );
        motor.brake();
    }

    if robot.positioning {
        robot.status = RobotStatus::ChaseTennis;

        if filtered_area > position_high {
            robot.grab_confirm_count = 0;
            eprintln!(
                "[control] positioning-too-close: filtered={filtered_area:.3} > {position_high:.3}; reverse-straight speed={NEAR_DISTANCE_SPEED} time={}ms",
                NEAR_MOVE_US / 1000
            );
            motor.drive(-NEAR_DISTANCE_SPEED, -NEAR_DISTANCE_SPEED);
            sleep_us(NEAR_MOVE_US);
            motor.brake();
            return;
        }

        if !centered {
            robot.grab_confirm_count = 0;
            perform_position_turn(offset, image_w, motor);
            return;
        }

        if filtered_area < position_low {
            robot.grab_confirm_count = 0;
            eprintln!(
                "[control] positioning-too-far: filtered={filtered_area:.3} < {position_low:.3}; forward-straight speed={NEAR_DISTANCE_SPEED} time={}ms",
                NEAR_MOVE_US / 1000
            );
            motor.drive(NEAR_DISTANCE_SPEED, NEAR_DISTANCE_SPEED);
            sleep_us(NEAR_MOVE_US);
            motor.brake();
            robot.last_motion_was_forward = true;
            return;
        }

        motor.brake();
        robot.status = RobotStatus::GrabTennis;
        robot.grab_confirm_count += 1;
        eprintln!(
            "[grab] confirm {}/{} positioning-distance=[{position_low:.3},{position_high:.3}]",
            robot.grab_confirm_count, GRAB_CONFIRM_THRESHOLD
        );

        if robot.grab_confirm_count >= GRAB_CONFIRM_THRESHOLD {
            eprintln!(
                "[grab] trigger area={area_ratio:.3} filtered={filtered_area:.3} cx={ball_cx} target={center} error={offset}"
            );
            motor.standby();
            arm.grab();
            sleep_us(2_000_00);
            arm.release();
            sleep_us(1_000_00);
            arm.grab_pos();
            sleep_us(1_000_00);

            robot.grab_confirm_count = 0;
            robot.status = RobotStatus::ChaseTennis;
            robot.reset_approach();
        }
        return;
    }

    if filtered_area < distance_low {
        robot.grab_confirm_count = 0;
        robot.status = RobotStatus::ChaseTennis;

        match robot.approach_phase {
            ApproachPhase::Fast => {
                if predicted_area >= fast_exit {
                    let next_phase = if predicted_area >= medium_exit {
                        ApproachPhase::Fine
                    } else {
                        ApproachPhase::Medium
                    };
                    robot.enter_approach_phase(next_phase);
                    eprintln!(
                        "[control] phase-brake: Fast -> {:?} area={area_ratio:.3} predicted={predicted_area:.3}",
                        robot.approach_phase
                    );
                    motor.brake();
                    return;
                }
                drive_continuous_approach(
                    "fast-approach",
                    offset,
                    image_w,
                    FAR_DISTANCE_SPEED,
                    FAR_ARC_TURN_DELTA,
                    robot,
                    motor,
                );
            }
            ApproachPhase::Medium => {
                if predicted_area >= medium_exit {
                    robot.enter_approach_phase(ApproachPhase::Fine);
                    eprintln!(
                        "[control] phase-brake: Medium -> Fine area={area_ratio:.3} predicted={predicted_area:.3}"
                    );
                    motor.brake();
                    return;
                }
                drive_continuous_approach(
                    "medium-approach",
                    offset,
                    image_w,
                    MEDIUM_DISTANCE_SPEED,
                    MEDIUM_ARC_TURN_DELTA,
                    robot,
                    motor,
                );
            }
            ApproachPhase::Fine => {
                if predicted_area >= distance_low {
                    eprintln!(
                        "[control] predictive-brake: area={area_ratio:.3} predicted={predicted_area:.3} target={stop_area:.3}"
                    );
                    motor.brake();
                    return;
                }

                if !centered && robot.ineffective_alignment_count >= STUCK_ACTION_THRESHOLD {
                    perform_strong_turn(offset, robot, motor);
                    return;
                }

                let steering = robot.update_steering(offset, image_w);
                let delta = proportional_turn_delta(offset, image_w, NEAR_ARC_TURN_DELTA, steering);
                let (left, right) = forward_steering_speeds(steering, NEAR_DISTANCE_SPEED, delta);
                eprintln!(
                    "[control] fine-approach: steering={steering:?} left={left} right={right} time={}ms area={area_ratio:.3} target={stop_area:.3}",
                    NEAR_MOVE_US / 1000
                );
                motor.drive(left, right);
                sleep_us(NEAR_MOVE_US);
                motor.brake();
                robot.last_motion_was_forward = true;
                if steering != SteeringState::Straight {
                    robot.remember_alignment_action(offset);
                }
            }
        }
        return;
    }
}

fn drive_continuous_approach(
    label: &str,
    offset: i32,
    image_w: i32,
    base_speed: i32,
    max_turn_delta: i32,
    robot: &mut RobotState,
    motor: &mut Motor,
) {
    let steering = robot.update_steering(offset, image_w);
    let delta = proportional_turn_delta(offset, image_w, max_turn_delta, steering);
    let (left, right) = forward_steering_speeds(steering, base_speed, delta);
    eprintln!(
        "[control] {label}: steering={steering:?} left={left} right={right} continuous error={offset}"
    );
    motor.drive(left, right);
    robot.last_motion_was_forward = true;
    if steering != SteeringState::Straight {
        robot.remember_alignment_action(offset);
    }
}

fn target_x(image_w: i32, center_offset_px: i32) -> i32 {
    let scaled_offset = (center_offset_px as f32 * image_w as f32 / REFERENCE_FRAME_WIDTH)
        .round()
        .clamp(-(image_w as f32), image_w as f32) as i32;
    (image_w / 2 + scaled_offset).clamp(0, image_w.saturating_sub(1))
}

fn approach_thresholds(distance_low: f32) -> (f32, f32) {
    let fast_exit = (distance_low * FAST_EXIT_RATIO).clamp(0.0, distance_low);
    let medium_exit = (distance_low * MEDIUM_EXIT_RATIO).clamp(fast_exit, distance_low);
    (fast_exit, medium_exit)
}

fn approach_center_offset(
    configured_offset: i32,
    phase: ApproachPhase,
    area_ratio: f32,
    fast_exit: f32,
    medium_exit: f32,
) -> i32 {
    match phase {
        // 远处先朝画面中心快速靠近，避免一开始就为夹爪偏置不断修方向。
        ApproachPhase::Fast => 0,
        // 中段平滑移向机械夹爪的真实目标点，避免目标一次横跳 70 像素。
        ApproachPhase::Medium => {
            let span = (medium_exit - fast_exit).max(f32::EPSILON);
            let progress = ((area_ratio - fast_exit) / span).clamp(0.0, 1.0);
            (configured_offset as f32 * progress).round() as i32
        }
        ApproachPhase::Fine => configured_offset,
    }
}

fn target_phase_for_area(
    current: ApproachPhase,
    area_ratio: f32,
    fast_exit: f32,
    medium_exit: f32,
    distance_low: f32,
) -> ApproachPhase {
    if area_ratio >= distance_low {
        ApproachPhase::Fine
    } else {
        match current {
            ApproachPhase::Fast if area_ratio >= medium_exit => ApproachPhase::Fine,
            ApproachPhase::Fast if area_ratio >= fast_exit => ApproachPhase::Medium,
            ApproachPhase::Medium if area_ratio >= medium_exit => ApproachPhase::Fine,
            phase => phase,
        }
    }
}

fn scaled_reference_px(reference_px: i32, image_w: i32) -> i32 {
    (reference_px as f32 * image_w as f32 / REFERENCE_FRAME_WIDTH)
        .round()
        .max(1.0) as i32
}

fn proportional_turn_delta(
    offset: i32,
    image_w: i32,
    max_delta: i32,
    steering: SteeringState,
) -> i32 {
    if steering == SteeringState::Straight || max_delta <= 0 {
        return 0;
    }
    let exit = scaled_reference_px(STEER_EXIT_PX, image_w);
    let full_scale = (image_w / 4).max(1);
    (((offset.abs() - exit).max(0) * max_delta) / full_scale).clamp(2, max_delta)
}

fn forward_steering_speeds(
    steering: SteeringState,
    base_speed: i32,
    turn_delta: i32,
) -> (i32, i32) {
    match steering {
        SteeringState::Straight => (base_speed, base_speed),
        SteeringState::Left => (base_speed - turn_delta, base_speed + turn_delta),
        SteeringState::Right => (base_speed + turn_delta, base_speed - turn_delta),
    }
}

fn perform_strong_turn(offset: i32, robot: &mut RobotState, motor: &mut Motor) {
    let (left, right) = if offset < 0 {
        (-STRONG_TURN_SPEED, STRONG_TURN_SPEED)
    } else {
        (STRONG_TURN_SPEED, -STRONG_TURN_SPEED)
    };
    eprintln!(
        "[control] strong-turn after slip: left={left} right={right} time={}ms error={offset}",
        STRONG_TURN_US / 1000
    );
    motor.drive(left, right);
    sleep_us(STRONG_TURN_US);
    motor.standby();
    robot.ineffective_alignment_count = 0;
    robot.remember_alignment_action(offset);
}

fn position_turn_us(offset: i32, image_w: i32) -> u64 {
    let margin = scaled_reference_px(GRAB_CENTER_MARGIN_PX, image_w);
    let full_scale = (image_w / 4).max(margin + 1);
    let progress = ((offset.abs() - margin).max(0) as f32 / (full_scale - margin).max(1) as f32)
        .clamp(0.0, 1.0);
    POSITION_TURN_MIN_US
        + ((POSITION_TURN_MAX_US - POSITION_TURN_MIN_US) as f32 * progress).round() as u64
}

fn perform_position_turn(offset: i32, image_w: i32, motor: &mut Motor) {
    let pulse_us = position_turn_us(offset, image_w);
    let (left, right) = if offset < 0 {
        (-POSITION_TURN_SPEED, POSITION_TURN_SPEED)
    } else {
        (POSITION_TURN_SPEED, -POSITION_TURN_SPEED)
    };
    eprintln!(
        "[control] positioning-turn: left={left} right={right} time={}ms error={offset}",
        pulse_us / 1000
    );
    motor.drive(left, right);
    sleep_us(pulse_us);
    motor.brake();
    sleep_us(POSITION_SETTLE_US);
}

fn sleep_us(us: u64) {
    thread::sleep(Duration::from_micros(us));
}

#[cfg(test)]
mod tests {
    use super::{
        approach_center_offset, approach_thresholds, forward_steering_speeds, position_turn_us,
        proportional_turn_delta, scaled_reference_px, target_phase_for_area, target_x,
        ApproachPhase, RobotState, SteeringState, POSITION_TURN_MAX_US, POSITION_TURN_MIN_US,
    };

    #[test]
    fn scales_shifted_target_and_margin() {
        assert_eq!(target_x(640, 50), 370);
        assert_eq!(target_x(320, 50), 185);
        assert_eq!(target_x(640, -50), 270);
        assert_eq!(scaled_reference_px(35, 640), 35);
        assert_eq!(scaled_reference_px(35, 320), 18);
    }

    #[test]
    fn arcs_turn_toward_target_in_both_directions() {
        assert_eq!(
            forward_steering_speeds(SteeringState::Left, 20, 6),
            (14, 26)
        );
        assert_eq!(
            forward_steering_speeds(SteeringState::Right, 20, 6),
            (26, 14)
        );
        assert_eq!(
            forward_steering_speeds(SteeringState::Straight, 20, 6),
            (20, 20)
        );
    }

    #[test]
    fn approach_thresholds_scale_with_stop_distance() {
        let (fast, medium) = approach_thresholds(0.48);
        assert!((fast - 0.336).abs() < 0.0001);
        assert!((medium - 0.432).abs() < 0.0001);
    }

    #[test]
    fn positioning_turn_is_short_but_overcomes_static_friction() {
        assert_eq!(position_turn_us(35, 640), POSITION_TURN_MIN_US);
        assert!(position_turn_us(100, 640) > POSITION_TURN_MIN_US);
        assert_eq!(position_turn_us(320, 640), POSITION_TURN_MAX_US);
    }

    #[test]
    fn target_moves_smoothly_from_camera_center_to_gripper_offset() {
        assert_eq!(
            approach_center_offset(70, ApproachPhase::Fast, 0.30, 0.52, 0.62),
            0
        );
        assert_eq!(
            approach_center_offset(70, ApproachPhase::Medium, 0.57, 0.52, 0.62),
            35
        );
        assert_eq!(
            approach_center_offset(70, ApproachPhase::Fine, 0.70, 0.52, 0.62),
            70
        );
    }

    #[test]
    fn sudden_area_jump_uses_final_gripper_target() {
        assert_eq!(
            target_phase_for_area(ApproachPhase::Fast, 0.74, 0.52, 0.62, 0.73),
            ApproachPhase::Fine
        );
        assert_eq!(
            target_phase_for_area(ApproachPhase::Fast, 0.57, 0.52, 0.62, 0.73),
            ApproachPhase::Medium
        );
    }

    #[test]
    fn steering_hysteresis_prevents_frame_by_frame_direction_flips() {
        let mut state = RobotState::default();
        assert_eq!(state.update_steering(50, 640), SteeringState::Straight);
        assert_eq!(state.update_steering(70, 640), SteeringState::Right);
        assert_eq!(state.update_steering(40, 640), SteeringState::Right);
        assert_eq!(state.update_steering(20, 640), SteeringState::Straight);
        assert_eq!(state.update_steering(-70, 640), SteeringState::Left);
    }

    #[test]
    fn steering_delta_is_proportional_and_capped() {
        assert_eq!(
            proportional_turn_delta(30, 640, 8, SteeringState::Straight),
            0
        );
        assert_eq!(proportional_turn_delta(70, 640, 8, SteeringState::Right), 2);
        assert_eq!(
            proportional_turn_delta(300, 640, 8, SteeringState::Right),
            8
        );
    }

    #[test]
    fn filters_area_with_three_sample_median() {
        let mut state = RobotState::default();
        assert!((state.filtered_area(0.10) - 0.10).abs() < f32::EPSILON);
        assert!((state.filtered_area(0.30) - 0.20).abs() < f32::EPSILON);
        assert!((state.filtered_area(0.20) - 0.20).abs() < f32::EPSILON);
        assert!((state.filtered_area(0.50) - 0.30).abs() < f32::EPSILON);
    }

    #[test]
    fn predicts_one_forward_frame_but_not_stationary_growth() {
        let mut state = RobotState::default();
        assert!((state.predicted_area(0.60) - 0.60).abs() < f32::EPSILON);
        state.last_motion_was_forward = true;
        assert!((state.predicted_area(0.66) - 0.72).abs() < 0.0001);
        assert!((state.predicted_area(0.70) - 0.70).abs() < 0.0001);
    }
}
