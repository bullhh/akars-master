use akars::arm::Arm;
use akars::arm_config::{ArmCalibration, ArmPose, ArmPoseName};
use akars::camera::UsbCamera;
use akars::motor::{Motor, MotorConfig};
use akars::robot::{install_signal_handlers, run_tennis_hunter, RobotConfig};
use akars::tpu::{open_model, InferenceConfig};
use akars::web::{serve, WebConfig};
use std::env;
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;

#[derive(Debug)]
struct Cli {
    model: PathBuf,
    camera: String,
    motor: String,
    arm: String,
    arm_config: PathBuf,
    frames: Option<u64>,
    classes: i32,
    conf: f32,
    iou: f32,
}

/// Arguments for the standalone TPU inference test: run a model on a single
/// image file and write an annotated result image.
#[derive(Debug)]
struct DetectCli {
    model: PathBuf,
    input: PathBuf,
    output: PathBuf,
    classes: i32,
    conf: f32,
    iou: f32,
}

/// Arguments for the camera capture test: grab a single frame and save it,
/// discarding the first few frames so the sensor's exposure/white-balance can
/// settle (warm up).
#[derive(Debug)]
struct CaptureCli {
    camera: String,
    output: PathBuf,
    warmup: u32,
}

/// Arguments for manual arm debugging: set all three servo angles directly.
#[derive(Debug)]
struct ArmTestCli {
    arm: String,
    config: PathBuf,
    save_pose: Option<ArmPoseName>,
    servo0: f32,
    servo1: f32,
    servo2: f32,
    time_ms: i32,
}

/// Arguments for replaying one persisted arm pose.
#[derive(Debug)]
struct ArmPoseCli {
    arm: String,
    config: PathBuf,
    pose: ArmPoseName,
    time_ms: i32,
}

/// Arguments for printing the persisted arm calibration.
#[derive(Debug)]
struct ArmShowCli {
    config: PathBuf,
}

/// Arguments for persisting the visual stop-distance threshold.
#[derive(Debug)]
struct StopSetCli {
    config: PathBuf,
    stop_area: f32,
}

/// Arguments for persisting the horizontal grab-target offset.
#[derive(Debug)]
struct CenterSetCli {
    config: PathBuf,
    center_offset_px: i32,
}

#[derive(Debug)]
enum Command {
    Hunt(Cli),
    Serve(WebConfig),
    Detect(DetectCli),
    Capture(CaptureCli),
    ArmTest(ArmTestCli),
    ArmPose(ArmPoseCli),
    ArmShow(ArmShowCli),
    StopSet(StopSetCli),
    CenterSet(CenterSetCli),
}

impl Default for Cli {
    fn default() -> Self {
        Self {
            model: PathBuf::new(),
            camera: "/dev/cvi-usb-camera0".to_string(),
            motor: "/dev/ttyS1".to_string(),
            arm: "/dev/ttyS2".to_string(),
            arm_config: PathBuf::from("arm.conf"),
            frames: None,
            classes: 1,
            conf: 0.5,
            iou: 0.5,
        }
    }
}

impl Default for DetectCli {
    fn default() -> Self {
        Self {
            model: PathBuf::new(),
            input: PathBuf::new(),
            output: PathBuf::from("detect_out.jpg"),
            classes: 1,
            conf: 0.5,
            iou: 0.5,
        }
    }
}

impl Default for CaptureCli {
    fn default() -> Self {
        Self {
            camera: "/dev/cvi-usb-camera0".to_string(),
            output: PathBuf::from("capture.jpg"),
            warmup: 30,
        }
    }
}

impl Default for ArmTestCli {
    fn default() -> Self {
        Self {
            arm: "/dev/ttyS2".to_string(),
            config: PathBuf::from("arm.conf"),
            save_pose: None,
            servo0: 0.0,
            servo1: 0.0,
            servo2: 0.0,
            time_ms: 1000,
        }
    }
}

fn main() {
    let command = match parse_cli(env::args().skip(1)) {
        Ok(command) => command,
        Err(message) => {
            eprintln!("{message}");
            print_usage();
            std::process::exit(2);
        }
    };

    match command {
        Command::Hunt(cli) => {
            install_signal_handlers();
            run_hunt(cli);
        }
        Command::Serve(config) => {
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .expect("failed to build tokio runtime");
            if let Err(err) = runtime.block_on(serve(config)) {
                eprintln!("[web] server failed: {err}");
                std::process::exit(1);
            }
        }
        Command::Detect(cli) => run_detect(cli),
        Command::Capture(cli) => run_capture(cli),
        Command::ArmTest(cli) => run_armtest(cli),
        Command::ArmPose(cli) => run_armpose(cli),
        Command::ArmShow(cli) => run_armshow(cli),
        Command::StopSet(cli) => run_stopset(cli),
        Command::CenterSet(cli) => run_centerset(cli),
    }
}

fn run_armtest(cli: ArmTestCli) {
    let pose = match ArmPose::new(cli.servo0, cli.servo1, cli.servo2) {
        Ok(pose) => pose,
        Err(err) => {
            eprintln!("[armtest] invalid pose: {err}");
            std::process::exit(1);
        }
    };
    let mut arm = match Arm::open(&cli.arm, 115200) {
        Ok(arm) => arm,
        Err(err) => {
            eprintln!("[armtest] failed to open {}: {err}", cli.arm);
            std::process::exit(1);
        }
    };

    let time_ms = cli.time_ms.clamp(0, 10_000);
    arm.move_to_pose(pose, time_ms);

    eprintln!(
        "[armtest] sent: servo0={:.1} servo1={:.1} servo2={:.1} time_ms={}",
        cli.servo0, cli.servo1, cli.servo2, time_ms
    );

    if let Some(name) = cli.save_pose {
        let mut calibration = load_arm_calibration(&cli.config);
        calibration.set(name, pose);
        if let Err(err) = calibration.save(&cli.config) {
            eprintln!("[armcal] failed to save {}: {err}", cli.config.display());
            std::process::exit(1);
        }
        eprintln!("[armcal] saved {name}={pose} to {}", cli.config.display());
    }
}

fn run_armpose(cli: ArmPoseCli) {
    let calibration = load_arm_calibration(&cli.config);
    let pose = calibration.pose(cli.pose);
    let mut arm = match Arm::open(&cli.arm, 115200) {
        Ok(arm) => arm,
        Err(err) => {
            eprintln!("[armpose] failed to open {}: {err}", cli.arm);
            std::process::exit(1);
        }
    };
    let time_ms = cli.time_ms.clamp(0, 10_000);
    arm.move_to_pose(pose, time_ms);
    eprintln!(
        "[armpose] applied {}={} from {} in {} ms",
        cli.pose,
        pose,
        cli.config.display(),
        time_ms
    );
}

fn run_armshow(cli: ArmShowCli) {
    let calibration = load_arm_calibration(&cli.config);
    print!("{calibration}");
}

fn run_stopset(cli: StopSetCli) {
    let mut calibration = load_arm_calibration(&cli.config);
    if let Err(err) = calibration.set_stop_area(cli.stop_area) {
        eprintln!("[stopset] invalid stop_area: {err}");
        std::process::exit(1);
    }
    if let Err(err) = calibration.save(&cli.config) {
        eprintln!("[stopset] failed to save {}: {err}", cli.config.display());
        std::process::exit(1);
    }
    eprintln!(
        "[stopset] saved stop_area={:.3} to {} (larger means closer)",
        calibration.stop_area,
        cli.config.display()
    );
}

fn run_centerset(cli: CenterSetCli) {
    let mut calibration = load_arm_calibration(&cli.config);
    if let Err(err) = calibration.set_center_offset_px(cli.center_offset_px) {
        eprintln!("[centerset] invalid center_offset_px: {err}");
        std::process::exit(1);
    }
    if let Err(err) = calibration.save(&cli.config) {
        eprintln!("[centerset] failed to save {}: {err}", cli.config.display());
        std::process::exit(1);
    }
    eprintln!(
        "[centerset] saved center_offset_px={} to {} (positive means image right)",
        calibration.center_offset_px,
        cli.config.display()
    );
}

fn load_arm_calibration(path: &std::path::Path) -> ArmCalibration {
    match ArmCalibration::load(path) {
        Ok(calibration) => {
            eprintln!("[armcal] loaded {}", path.display());
            calibration
        }
        Err(err) if err.kind() == io::ErrorKind::NotFound => {
            eprintln!(
                "[armcal] {} not found; using built-in defaults",
                path.display()
            );
            ArmCalibration::default()
        }
        Err(err) => {
            eprintln!("[armcal] failed to load {}: {err}", path.display());
            std::process::exit(1);
        }
    }
}

fn run_hunt(cli: Cli) {
    let camera = match UsbCamera::open(&cli.camera) {
        Ok(camera) => {
            let info = camera.info();
            eprintln!(
                "[camera] opened {}: {}x{} format={} connected={}",
                cli.camera, info.width, info.height, info.format, info.connected
            );
            camera
        }
        Err(err) => {
            eprintln!("[camera] failed to open {}: {err}", cli.camera);
            std::process::exit(1);
        }
    };

    let model = match open_model(&cli.model) {
        Ok(model) => model,
        Err(err) => {
            eprintln!("[tpu] failed to open model {}: {err}", cli.model.display());
            std::process::exit(1);
        }
    };
    eprintln!("[dbg] model opened, opening motor {} ...", cli.motor);

    let motor_config = MotorConfig {
        device: cli.motor.clone(),
        ..MotorConfig::default()
    };
    let motor = match Motor::open(&motor_config) {
        Ok(motor) => motor,
        Err(err) => {
            eprintln!("[motor] failed to open {}: {err}", cli.motor);
            std::process::exit(1);
        }
    };
    eprintln!("[dbg] motor opened, opening arm {} ...", cli.arm);

    let arm_calibration = load_arm_calibration(&cli.arm_config);
    let arm = match Arm::open_with_calibration(&cli.arm, 115200, arm_calibration) {
        Ok(arm) => arm,
        Err(err) => {
            eprintln!("[arm] failed to open {}: {err}", cli.arm);
            std::process::exit(1);
        }
    };
    eprintln!("[dbg] arm opened, entering tennis hunter ...");

    let config = RobotConfig {
        inference: InferenceConfig {
            classes_num: cli.classes,
            confidence_threshold: cli.conf,
            iou_threshold: cli.iou,
        },
        max_frames: cli.frames,
        stop_area: arm_calibration.stop_area,
        center_offset_px: arm_calibration.center_offset_px,
    };

    run_tennis_hunter(camera, model, motor, arm, config);
}

fn run_detect(cli: DetectCli) {
    let image = match std::fs::read(&cli.input) {
        Ok(bytes) => bytes,
        Err(err) => {
            eprintln!("[detect] failed to read {}: {err}", cli.input.display());
            std::process::exit(1);
        }
    };

    let mut model = match open_model(&cli.model) {
        Ok(model) => model,
        Err(err) => {
            eprintln!("[tpu] failed to open model {}: {err}", cli.model.display());
            std::process::exit(1);
        }
    };

    let config = InferenceConfig {
        classes_num: cli.classes,
        confidence_threshold: cli.conf,
        iou_threshold: cli.iou,
    };

    match model.detect_image(&image, &cli.output, config) {
        Ok(detections) => {
            eprintln!(
                "[detect] {} detection(s) on {}",
                detections.len(),
                cli.input.display()
            );
            for (i, d) in detections.iter().enumerate() {
                eprintln!(
                    "  #{i} class={} score={:.3} box=(cx={:.1}, cy={:.1}, w={:.1}, h={:.1})",
                    d.cls, d.score, d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h
                );
            }
            eprintln!(
                "[detect] annotated image written to {}",
                cli.output.display()
            );
        }
        Err(err) => {
            eprintln!("[detect] inference failed: {err}");
            std::process::exit(1);
        }
    }
}

fn run_capture(cli: CaptureCli) {
    let mut camera = match UsbCamera::open(&cli.camera) {
        Ok(camera) => {
            let info = camera.info();
            eprintln!(
                "[capture] opened {}: {}x{} format={} connected={}",
                cli.camera, info.width, info.height, info.format, info.connected
            );
            camera
        }
        Err(err) => {
            eprintln!("[capture] failed to open {}: {err}", cli.camera);
            std::process::exit(1);
        }
    };

    // Discard the first few frames so the sensor's auto exposure / white
    // balance can settle before we keep one.
    for i in 0..cli.warmup {
        if let Err(err) = camera.get_frame() {
            eprintln!("[capture] warm-up frame {i} failed: {err}");
            std::process::exit(1);
        }
    }
    if cli.warmup > 0 {
        eprintln!("[capture] discarded {} warm-up frame(s)", cli.warmup);
    }

    let frame = match camera.get_frame() {
        Ok(frame) => frame,
        Err(err) => {
            eprintln!("[capture] failed to capture frame: {err}");
            std::process::exit(1);
        }
    };

    if let Err(err) = akars::image_bridge::save_yuv422p(
        &frame.pixels,
        frame.width as i32,
        frame.height as i32,
        &cli.output,
    ) {
        eprintln!("[capture] failed to write {}: {err}", cli.output.display());
        std::process::exit(1);
    }

    eprintln!(
        "[capture] wrote {}x{} frame to {}",
        frame.width,
        frame.height,
        cli.output.display()
    );
}

fn parse_cli(args: impl Iterator<Item = String>) -> Result<Command, String> {
    let mut args = args.peekable();
    match args.peek().map(String::as_str) {
        Some("serve") => {
            args.next();
            parse_serve_cli(args).map(Command::Serve)
        }
        Some("detect") => {
            args.next();
            parse_detect_cli(args).map(Command::Detect)
        }
        Some("capture") => {
            args.next();
            parse_capture_cli(args).map(Command::Capture)
        }
        Some("armtest") => {
            args.next();
            parse_armtest_cli(args).map(Command::ArmTest)
        }
        Some("armpose") => {
            args.next();
            parse_armpose_cli(args).map(Command::ArmPose)
        }
        Some("armshow") => {
            args.next();
            parse_armshow_cli(args).map(Command::ArmShow)
        }
        Some("stopset") => {
            args.next();
            parse_stopset_cli(args).map(Command::StopSet)
        }
        Some("centerset") => {
            args.next();
            parse_centerset_cli(args).map(Command::CenterSet)
        }
        _ => parse_hunt_cli(args).map(Command::Hunt),
    }
}

fn parse_armtest_cli(args: impl Iterator<Item = String>) -> Result<ArmTestCli, String> {
    let mut cli = ArmTestCli::default();
    let mut args = args.peekable();
    let mut positional: Vec<f32> = Vec::with_capacity(3);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_armtest_usage();
                std::process::exit(0);
            }
            "--arm" => cli.arm = take_value(&mut args, "--arm")?,
            "--config" => cli.config = PathBuf::from(take_value(&mut args, "--config")?),
            "--save" => {
                cli.save_pose = Some(take_value(&mut args, "--save")?.parse::<ArmPoseName>()?);
            }
            "--time" => {
                cli.time_ms = take_value(&mut args, "--time")?
                    .parse()
                    .map_err(|_| "--time expects an integer (ms)".to_string())?;
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown armtest option: {value}"))
            }
            value => {
                if positional.len() >= 3 {
                    return Err(format!("unexpected armtest argument: {value}"));
                }
                let angle = value
                    .parse::<f32>()
                    .map_err(|_| format!("invalid angle value: {value}"))?;
                positional.push(angle);
            }
        }
    }

    if positional.len() != 3 {
        return Err(
            "armtest expects exactly 3 angle arguments: <servo0> <servo1> <servo2>".to_string(),
        );
    }

    cli.servo0 = positional[0];
    cli.servo1 = positional[1];
    cli.servo2 = positional[2];
    Ok(cli)
}

fn parse_armpose_cli(args: impl Iterator<Item = String>) -> Result<ArmPoseCli, String> {
    let mut arm = "/dev/ttyS2".to_string();
    let mut config = PathBuf::from("arm.conf");
    let mut pose = None;
    let mut time_ms = 1000;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_armpose_usage();
                std::process::exit(0);
            }
            "--arm" => arm = take_value(&mut args, "--arm")?,
            "--config" => config = PathBuf::from(take_value(&mut args, "--config")?),
            "--time" => {
                time_ms = take_value(&mut args, "--time")?
                    .parse()
                    .map_err(|_| "--time expects an integer (ms)".to_string())?;
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown armpose option: {value}"));
            }
            value if pose.is_none() => pose = Some(value.parse::<ArmPoseName>()?),
            value => return Err(format!("unexpected armpose argument: {value}")),
        }
    }
    Ok(ArmPoseCli {
        arm,
        config,
        pose: pose.ok_or_else(|| "armpose requires a pose name".to_string())?,
        time_ms,
    })
}

fn parse_armshow_cli(args: impl Iterator<Item = String>) -> Result<ArmShowCli, String> {
    let mut config = PathBuf::from("arm.conf");
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_armshow_usage();
                std::process::exit(0);
            }
            "--config" => config = PathBuf::from(take_value(&mut args, "--config")?),
            value => return Err(format!("unexpected armshow argument: {value}")),
        }
    }
    Ok(ArmShowCli { config })
}

fn parse_stopset_cli(args: impl Iterator<Item = String>) -> Result<StopSetCli, String> {
    let mut config = PathBuf::from("arm.conf");
    let mut stop_area = None;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_stopset_usage();
                std::process::exit(0);
            }
            "--config" => config = PathBuf::from(take_value(&mut args, "--config")?),
            value if value.starts_with('-') => {
                return Err(format!("unknown stopset option: {value}"));
            }
            value if stop_area.is_none() => {
                stop_area = Some(
                    value
                        .parse::<f32>()
                        .map_err(|_| format!("invalid stop_area value: {value}"))?,
                );
            }
            value => return Err(format!("unexpected stopset argument: {value}")),
        }
    }
    Ok(StopSetCli {
        config,
        stop_area: stop_area.ok_or_else(|| "stopset requires a stop_area value".to_string())?,
    })
}

fn parse_centerset_cli(args: impl Iterator<Item = String>) -> Result<CenterSetCli, String> {
    let mut config = PathBuf::from("arm.conf");
    let mut center_offset_px = None;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_centerset_usage();
                std::process::exit(0);
            }
            "--config" => config = PathBuf::from(take_value(&mut args, "--config")?),
            value if center_offset_px.is_none() => {
                center_offset_px = Some(
                    value
                        .parse::<i32>()
                        .map_err(|_| format!("invalid center_offset_px value: {value}"))?,
                );
            }
            value => return Err(format!("unexpected centerset argument: {value}")),
        }
    }
    Ok(CenterSetCli {
        config,
        center_offset_px: center_offset_px
            .ok_or_else(|| "centerset requires a center_offset_px value".to_string())?,
    })
}

fn parse_hunt_cli(args: impl Iterator<Item = String>) -> Result<Cli, String> {
    let mut cli = Cli::default();
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            "--camera" => cli.camera = take_value(&mut args, "--camera")?,
            "--motor" => cli.motor = take_value(&mut args, "--motor")?,
            "--arm" => cli.arm = take_value(&mut args, "--arm")?,
            "--arm-config" => {
                cli.arm_config = PathBuf::from(take_value(&mut args, "--arm-config")?)
            }
            "--frames" => {
                cli.frames = Some(
                    take_value(&mut args, "--frames")?
                        .parse()
                        .map_err(|_| "--frames expects an integer".to_string())?,
                );
            }
            "--classes" => {
                cli.classes = take_value(&mut args, "--classes")?
                    .parse()
                    .map_err(|_| "--classes expects an integer".to_string())?;
            }
            "--conf" => {
                cli.conf = take_value(&mut args, "--conf")?
                    .parse()
                    .map_err(|_| "--conf expects a float".to_string())?;
            }
            "--iou" => {
                cli.iou = take_value(&mut args, "--iou")?
                    .parse()
                    .map_err(|_| "--iou expects a float".to_string())?;
            }
            value if value.starts_with('-') => return Err(format!("unknown option: {value}")),
            value => {
                if cli.model.as_os_str().is_empty() {
                    cli.model = PathBuf::from(value);
                } else {
                    return Err(format!("unexpected positional argument: {value}"));
                }
            }
        }
    }

    if cli.model.as_os_str().is_empty() {
        return Err("missing cvimodel path".to_string());
    }
    Ok(cli)
}

fn parse_serve_cli(args: impl Iterator<Item = String>) -> Result<WebConfig, String> {
    let mut config = WebConfig::default();
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_web_usage();
                std::process::exit(0);
            }
            "--listen" => {
                config.listen = take_value(&mut args, "--listen")?
                    .parse::<SocketAddr>()
                    .map_err(|_| {
                        "--listen expects HOST:PORT, for example 0.0.0.0:8080".to_string()
                    })?;
            }
            "--motor" => config.motor_device = take_value(&mut args, "--motor")?,
            "--arm" => config.arm_device = take_value(&mut args, "--arm")?,
            "--arm-config" => {
                config.arm_config = PathBuf::from(take_value(&mut args, "--arm-config")?)
            }
            "--mock" => config.mock = true,
            value if value.starts_with('-') => {
                return Err(format!("unknown serve option: {value}"))
            }
            value => return Err(format!("unexpected serve argument: {value}")),
        }
    }
    Ok(config)
}

fn parse_detect_cli(args: impl Iterator<Item = String>) -> Result<DetectCli, String> {
    let mut cli = DetectCli::default();
    let mut positional = 0;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_detect_usage();
                std::process::exit(0);
            }
            "-o" | "--out" => cli.output = PathBuf::from(take_value(&mut args, "--out")?),
            "--classes" => {
                cli.classes = take_value(&mut args, "--classes")?
                    .parse()
                    .map_err(|_| "--classes expects an integer".to_string())?;
            }
            "--conf" => {
                cli.conf = take_value(&mut args, "--conf")?
                    .parse()
                    .map_err(|_| "--conf expects a float".to_string())?;
            }
            "--iou" => {
                cli.iou = take_value(&mut args, "--iou")?
                    .parse()
                    .map_err(|_| "--iou expects a float".to_string())?;
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown detect option: {value}"))
            }
            value => {
                match positional {
                    0 => cli.model = PathBuf::from(value),
                    1 => cli.input = PathBuf::from(value),
                    _ => return Err(format!("unexpected detect argument: {value}")),
                }
                positional += 1;
            }
        }
    }

    if cli.model.as_os_str().is_empty() {
        return Err("missing cvimodel path".to_string());
    }
    if cli.input.as_os_str().is_empty() {
        return Err("missing input image path".to_string());
    }
    Ok(cli)
}

fn parse_capture_cli(args: impl Iterator<Item = String>) -> Result<CaptureCli, String> {
    let mut cli = CaptureCli::default();
    let mut positional = 0;
    let mut args = args.peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_capture_usage();
                std::process::exit(0);
            }
            "--camera" => cli.camera = take_value(&mut args, "--camera")?,
            "-o" | "--out" => cli.output = PathBuf::from(take_value(&mut args, "--out")?),
            "--warmup" => {
                cli.warmup = take_value(&mut args, "--warmup")?
                    .parse()
                    .map_err(|_| "--warmup expects a non-negative integer".to_string())?;
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown capture option: {value}"))
            }
            value => {
                match positional {
                    0 => cli.output = PathBuf::from(value),
                    _ => return Err(format!("unexpected capture argument: {value}")),
                }
                positional += 1;
            }
        }
    }
    Ok(cli)
}

fn take_value(
    args: &mut std::iter::Peekable<impl Iterator<Item = String>>,
    option: &str,
) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{option} expects a value"))
}

fn print_usage() {
    eprintln!(
        "Usage:\n  akars <model.cvimodel> [--camera DEV] [--motor DEV] [--arm DEV] [--arm-config PATH] [--frames N] [--classes N] [--conf X] [--iou X]\n  akars serve [--listen HOST:PORT] [--motor DEV] [--arm DEV] [--arm-config PATH] [--mock]\n  akars detect <model.cvimodel> <image> [--out PATH] [--classes N] [--conf X] [--iou X]\n  akars capture [output.jpg] [--camera DEV] [--out PATH] [--warmup N]\n  akars armtest <servo0> <servo1> <servo2> [--arm DEV] [--time MS] [--save POSE] [--config PATH]\n  akars armpose <ready|approach|closed|lift> [--arm DEV] [--time MS] [--config PATH]\n  akars armshow [--config PATH]\n  akars stopset <0.05..0.90> [--config PATH]\n  akars centerset <-240..240> [--config PATH]"
    );
}

fn print_capture_usage() {
    eprintln!(
        "Usage: akars capture [output.jpg] [--camera DEV] [--out PATH] [--warmup N]\n\nGrabs a single JPEG frame from the camera, discarding the first N frames so the\nsensor's auto exposure / white balance can settle (warm up).\n\nDefaults:\n  --camera /dev/cvi-usb-camera0\n  --out capture.jpg\n  --warmup 30"
    );
}

fn print_detect_usage() {
    eprintln!(
        "Usage: akars detect <model.cvimodel> <image> [--out PATH] [--classes N] [--conf X] [--iou X]\n\nRuns the TPU model on a single image and writes a copy with detection boxes drawn.\n\nDefaults:\n  --out detect_out.jpg\n  --classes 1\n  --conf 0.5\n  --iou 0.5"
    );
}

fn print_web_usage() {
    eprintln!(
        "Usage: akars serve [--listen HOST:PORT] [--motor DEV] [--arm DEV] [--arm-config PATH] [--mock]\n\nDefaults:\n  --listen 0.0.0.0:8080\n  --motor /dev/ttyS1\n  --arm /dev/ttyS2\n  --arm-config arm.conf"
    );
}

fn print_armtest_usage() {
    eprintln!(
        "Usage: akars armtest <servo0> <servo1> <servo2> [--arm DEV] [--time MS] [--save POSE] [--config PATH]\n\nMoves all three servos to explicit angles. After physically verifying a pose, rerun with --save ready|approach|closed|lift to persist it.\n\nDefaults:\n  --arm /dev/ttyS2\n  --time 1000\n  --config arm.conf"
    );
}

fn print_armpose_usage() {
    eprintln!(
        "Usage: akars armpose <ready|approach|closed|lift> [--arm DEV] [--time MS] [--config PATH]\n\nLoads one saved pose and applies it to the arm."
    );
}

fn print_armshow_usage() {
    eprintln!("Usage: akars armshow [--config PATH]\n\nPrints the effective arm calibration.");
}

fn print_stopset_usage() {
    eprintln!(
        "Usage: akars stopset <0.05..0.90> [--config PATH]\n\nPersists the visual stop distance. A larger value stops closer to the ball. The tuned default is 0.50."
    );
}

fn print_centerset_usage() {
    eprintln!(
        "Usage: akars centerset <-240..240> [--config PATH]\n\nPersists the horizontal grab target in reference-frame pixels. Positive values place the ball to the image right."
    );
}

#[cfg(test)]
mod tests {
    use super::{Cli, WebConfig};

    #[test]
    fn uses_board_device_defaults() {
        let hunt = Cli::default();
        assert_eq!(hunt.camera, "/dev/cvi-usb-camera0");
        assert_eq!(hunt.motor, "/dev/ttyS1");
        assert_eq!(hunt.arm, "/dev/ttyS2");

        let web = WebConfig::default();
        assert_eq!(web.motor_device, "/dev/ttyS1");
        assert_eq!(web.arm_device, "/dev/ttyS2");
    }
}
