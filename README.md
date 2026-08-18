# AKARS 两轮捡球机器人使用与调试说明

AKARS 是运行在 SG2002/StarryOS 用户态的两轮捡球程序，包含 USB 摄像头采集、
CVI TPU YOLOv8 网球检测、ESP32 电机串口控制、ZP10D 机械臂串口控制，以及
“发现球—接近—停车—抓取”的闭环状态机。

本文以板端部署目录 `/root/akars-debug` 为准。镜像原有的 `/root/akars` 和
`/root/run.sh` 是对比基线，不要覆盖。

## 1. 默认硬件和参数

当前机器的设备连接如下：

| 功能 | 默认设备 | 说明 |
| --- | --- | --- |
| USB 摄像头 | `/dev/cvi-usb-camera0` | 采集网球画面 |
| 左右轮控制器 | `/dev/ttyS1` | SG2002 通过 UART1 连接 ESP32 |
| 机械臂 | `/dev/ttyS2` | SG2002 通过 UART2 连接舵机控制板 |
| 模型 | `models/yolov8n_tennis_v4.cvimodel` | 默认单类别网球模型 |
| 标定文件 | `arm.conf` | 保存机械臂姿态、停车距离和水平目标 |

当前实车确认可抓球的默认标定为：

```ini
ready=150.0,100.0,140.0
approach=227.0,105.0,150.0
closed=227.0,105.0,80.0
lift=150.0,100.0,60.0
stop_area=0.480
center_offset_px=68
```

模型后处理默认值为：

```text
classes=1
conf=0.5
iou=0.5
```

这些默认值已固化，正常运行时不需要逐项输入。

## 2. 一键运行

将机器人放到安全、开阔的测试区域，确认车轮和机械臂周围无人、无线缆阻挡，
然后执行：

```sh
cd /root/akars-debug
./run.sh
```

无参数的 `run.sh` 会自动使用默认模型、摄像头、电机串口、机械臂串口、
`arm.conf` 和模型后处理参数。程序保持在前台运行，按 `Ctrl-C` 停止；收到退出
信号后程序会向电机发送停车命令。

需要临时覆盖默认值时，仍可使用完整命令：

```sh
./run.sh models/yolov8n_tennis_v4.cvimodel \
  --camera /dev/cvi-usb-camera0 \
  --motor /dev/ttyS1 \
  --arm /dev/ttyS2 \
  --arm-config arm.conf \
  --classes 1 \
  --conf 0.5 \
  --iou 0.5
```

## 3. 命令参数统一说明

完整捡球命令的格式为：

```sh
./run.sh <模型文件> [--camera 设备] [--motor 设备] [--arm 设备] \
  [--arm-config 配置文件] [--classes 数量] [--conf 阈值] [--iou 阈值]
```

各参数含义如下：

| 参数 | 当前默认值 | 含义 | 什么时候需要修改 |
| --- | --- | --- | --- |
| `./run.sh` | 无 | 板端启动脚本，设置运行库路径后启动 `akars` | 不修改 |
| `<模型文件>` | `models/yolov8n_tennis_v4.cvimodel` | TPU 使用的网球检测模型；它是位置参数，不带 `--` | 更换模型时 |
| `--camera` | `/dev/cvi-usb-camera0` | USB 摄像头设备节点 | 摄像头设备名改变时 |
| `--motor` | `/dev/ttyS1` | ESP32 左右轮控制器的串口 | 电机控制器改接其他串口时 |
| `--arm` | `/dev/ttyS2` | 机械臂舵机控制板的串口 | 机械臂改接其他串口时 |
| `--arm-config` | `arm.conf` | 机械臂姿态、停车距离和水平偏移的配置文件 | 对比另一份标定时 |
| `--classes` | `1` | 模型输出的类别数量；当前模型只有“网球”一类 | 换成多类别模型时 |
| `--conf` | `0.5` | 最低检测置信度；增大会减少误检，也可能漏掉不清晰的球 | 调整模型检出灵敏度时 |
| `--iou` | `0.5` | 非极大值抑制（NMS）的框重叠阈值；越小，重叠框抑制越强 | 同一个球出现多个框时 |

当前硬件和模型均已固定，因此平时直接执行 `./run.sh` 即可。完整命令主要用于
换模型、换设备或对比不同参数。

机械臂角度命令的格式为：

```sh
./run.sh armtest <servo0> <servo1> <servo2> \
  [--time 毫秒] [--save 姿态名] [--arm 设备] [--config 配置文件]
```

以这条命令为例：

```sh
./run.sh armtest 227 105 150 --time 1000 --save approach
```

| 参数 | 含义 |
| --- | --- |
| `armtest` | 进入机械臂直接角度测试模式，不启动摄像头和车轮 |
| `227` | `servo0` 的目标角度，允许范围 `0～270` 度 |
| `105` | `servo1` 的目标角度，允许范围 `0～270` 度 |
| `150` | `servo2` 的目标角度，允许范围 `0～270` 度 |
| `--time 1000` | 舵机完成本次动作的目标时间为 1000 ms；数值越大，动作通常越慢 |
| `--save approach` | 下发角度后，把同一组数值保存为 `approach` 姿态 |
| `--arm` | 可选，省略时使用 `/dev/ttyS2` |
| `--config` | 可选，省略时保存到当前目录的 `arm.conf` |

`--save` 可使用 `ready`、`approach`、`closed`、`lift`。安全的调试方式是先不带
`--save` 观察实际动作，确认角度合适后再增加 `--save` 固化。

## 4. 程序如何接近和停车

程序使用检测框面积占画面的比例估算距离。日志里的 `area` 是本帧原始面积，
`filtered` 是最近三帧面积滤波值；实际距离控制使用 `filtered`，并在车轮刚向前
运动后根据相邻两帧滤波面积增长量做一次前向预测。球的横坐标另做低通滤波，
防止检测框轻微抖动立即改变车轮方向。

默认 `stop_area=0.480`。接近阶段在滤波面积达到 `0.460` 时进入最终定位：

| `area` 范围 | 判定 | 动作 |
| --- | --- | --- |
| `< 约 0.230` | 球较远 | 速度 `45` 连续前进，弱弧线调整方向 |
| `约 0.230～0.299` | 中距离 | 提前制动，再以速度 `32` 连续接近 |
| `约 0.299～0.460` | 近距离 | 再次提前制动，以速度 `25`、时长 `90 ms` 的短脉冲接近 |
| `0.430～0.530` | 最终定位滞回范围 | 保持定位状态；距离合适时只修正水平位置 |
| `< 0.430` | 定位后明显过远 | 先静止确认是否确实较远，否则短脉冲前进 |
| `> 0.530` | 定位后明显过近 | 以速度 `25` 直线后退 `90 ms`，本帧禁止抓取 |

高速和中速出口按停车阈值的比例计算，并根据实车制动后仍会前移约一个识别周期
的现象提前降速。程序在高速、中速和精细阶段之间各主动制动一次，先消掉惯性再降速；预测
下一帧可能越过阶段边界或进入定位区时也会提前制动。

进入最终定位后，不会因 `area` 在 `0.460` 附近轻微波动立即退出。只有连续4个
有效帧都低于远距离阈值、球框不在画面边缘时，程序才在刹车静止确认后退回
Medium；随后还要等待近期近距离锁定消退，并再确认3个远距离帧，才允许恢复
Fast。不会从定位状态直接跳到 Fast，也不会清除用于提前刹车的面积增长预测。
水平位置不合格时使用双轮反向、速度 `30`、约 `60～100 ms` 的原地转向，然后
制动并重新观察。

高速前进时，左右轮差速不再固定为 `33/57`，而是随水平误差按比例变化，最大
差速修正为 `8`。球的误差超过 `60` 像素才开始转向，回到 `30` 像素内才退出
转向；这个进入/退出滞回可以避免连续帧在左右方向间反复切换。

`center_offset_px=68` 表示目标不在画面正中心，而是在中心右侧。以 640 像素宽
画面为例：

```text
目标位置 = 640 / 2 + 68 = 388
```

这样可以补偿夹爪只向一侧张开的机械结构。为避免远处就追着这个偏置目标左右
摆动，高速阶段先以画面中心为目标，中速阶段将目标逐渐从中心移到右侧 68 像素，
精细接近和抓取阶段才使用完整偏移。其他分辨率会等比例缩放这些像素值。球中心
与最终目标位置相差不超过 35 像素时，视为水平位置合格。

如果连续两次转向后，水平误差改善都不足 8 像素，程序认为车轮可能因小幅动作
而打滑，会升级为速度 `30`、持续 `150 ms` 的强力原地转向。

## 5. 查看和备份当前配置

查看当前有效配置不会打开电机串口，也不会驱动车轮：

```sh
cd /root/akars-debug
./run.sh armshow
```

调试前建议备份：

```sh
cp arm.conf arm.conf.good
sync
```

如果调整失败，可恢复：

```sh
cp arm.conf.good arm.conf
sync
./run.sh armshow
```

## 6. 机械臂角度调试

调试机械臂时建议将车轮架空或断开车轮动力，并让夹爪远离人手和硬物。以下
机械臂调试命令只打开 `/dev/ttyS2`，不会打开电机串口。

四组姿态的含义：

| 姿态 | 作用 |
| --- | --- |
| `ready` | 等待抓球，以及一次抓取完成后的复位姿态 |
| `approach` | 夹爪张开并下降到球附近 |
| `closed` | 保持机械臂位置并闭合夹爪 |
| `lift` | 保持夹紧并将球抬起 |

先单独回放当前固化姿态：

```sh
./run.sh armpose ready    --time 1000
./run.sh armpose approach --time 1000
./run.sh armpose closed   --time 1000
./run.sh armpose lift     --time 1000
```

尝试一组新角度但不保存：

```sh
./run.sh armtest 227 105 150 --time 1000
```

三个数字依次是 `servo0 servo1 servo2`，允许范围是 `0～270` 度。建议一次只改
一个舵机，每次改变 `2～5` 度，确认实际运动方向后再继续，避免大跨度动作造成
机械干涉或舵机堵转。

确认某组角度后，使用 `--save` 固化。例如保存 `approach`：

```sh
./run.sh armtest 227 105 150 --time 1000 --save approach
sync
./run.sh armshow
```

`--save` 可选值为 `ready`、`approach`、`closed`、`lift`。

推荐调试顺序：

1. 调好 `ready`，保证待机时机械臂不会碰地或挡住摄像头。
2. 调好 `approach`，使张开的夹爪能够罩住球。
3. 调好 `closed`，使夹爪能夹住球但不过度堵转。
4. 调好 `lift`，确保夹紧状态下能把球抬离地面。
5. 依次执行四条 `armpose` 命令复查，再开始整车测试。

## 7. 调整停车距离

使用 `stopset` 修改并立即保存停车阈值：

```sh
./run.sh stopset 0.48
./run.sh armshow
sync
```

调整原则：

- 停车太远、机械臂够不到球：逐步增大，例如从 `0.48` 改为 `0.50`。
- 球太近、机械臂下落会碰球：逐步减小，例如从 `0.48` 改为 `0.46`。
- 每次建议只改 `0.01～0.02`，观察多次测试，不要一次大幅改变。

面积比例不仅与物理距离有关，也受检测框大小、摄像头角度和球在画面中的位置
影响，因此不能把 `0.48` 直接理解为固定厘米数。

## 8. 调整球的左右位置

使用 `centerset` 修改并保存水平目标：

```sh
./run.sh centerset 68
./run.sh armshow
sync
```

调整原则以摄像头画面方向为准：

- 需要让球在画面中更靠右：增大，例如 `68 → 78`。
- 需要让球在画面中更靠左：减小，例如 `68 → 58`。
- 每次建议改变 5～10 像素，并保持停车距离不变进行对比。

`centerset` 只修改配置文件，不打开电机和机械臂串口。

## 9. 运行日志怎么看

典型检测日志如下：

```text
[detect] phase=Fine positioning=true area=0.482 filtered=0.478 predicted=0.483 cx_raw=390 cx=388 target=388 error=0 margin=35 conf=0.938 centered=true near_lock=8
```

字段含义：

- `area`：当前帧检测框面积比例。
- `filtered`：三帧中值面积比例，用于观察检测抖动。
- `predicted`：车轮刚向前运动时的一帧预测面积，用于提前制动。
- `phase`：当前接近阶段，`Fast`、`Medium` 或 `Fine`。
- `positioning`：是否已经锁定进入最终定位状态。
- `cx_raw`：本帧检测到的原始球中心横坐标。
- `cx`：低通滤波后的球中心横坐标，实际转向使用该值。
- `target`：考虑 `center_offset_px` 后的目标横坐标。
- `error`：`cx - target`；负数表示球在目标左侧，正数表示在目标右侧。
- `margin`：允许的水平误差。
- `conf`：模型检测置信度。
- `centered`：水平位置是否进入允许范围。

主要控制日志：

- `fast-approach`：远距离速度 45 连续前进，按比例做弱转向。
- `medium-approach`：中距离速度 32 连续前进，目标点逐渐右移。
- `fine-approach`：近停车带，正在执行速度 28、70 ms 的前进脉冲。
- `phase-brake`：进入下一接近阶段前主动制动，消除上一阶段惯性。
- `predictive-brake`：预测下一帧将进入停车带，已经提前制动。
- `enter-positioning`：首次进入最终定位并锁定该状态。
- `far-confirm`：检测结果突然变远，已刹车并收集连续静止帧确认，期间禁止加速。
- `far-recovery ... -> Medium`：确认确实较远，只恢复到中速连续前进。
- `far-recovery ... -> Fast`：近期近距离锁定已经消退，再次确认较远后恢复高速。
- `positioning-turn`：距离已进入定位范围，正在用双轮反向修正水平位置。
- `positioning-too-far`：锁定定位后明显过远，正在直线短脉冲前进。
- `positioning-too-close`：锁定定位后明显过近，正在直线短脉冲后退。
- `alignment ineffective`：上一次对准动作改善不足 8 像素。
- `strong-turn after slip`：连续小幅动作无效，正在强力转向克服打滑。
- `ball lost, searching`：没有检测到球，以速度28原地旋转搜索。
- 相机、预处理或推理失败：程序立即向左右轮发送刹车命令，禁止沿用旧的连续速度。
- `[grab] confirm N/2`：距离和水平位置连续满足，正在进行快速抓取确认。
- `[grab] trigger`：确认完成，开始执行机械臂抓取序列。

## 10. 分步排查

### 10.1 摄像头

只采集一张图片，不打开电机和机械臂：

```sh
./run.sh capture capture.jpg --camera /dev/cvi-usb-camera0 --warmup 30
```

如果出现连续 `I/O error (os error 5)`，先停止程序，检查 Type-C 摄像头连接和
供电，再重新启动。不要在错误循环中长时间运行。

### 10.2 TPU 模型

使用已有图片单独验证模型：

```sh
./run.sh detect models/yolov8n_tennis_v4.cvimodel capture.jpg \
  --out detect-out.jpg --classes 1 --conf 0.5 --iou 0.5
```

这一步不驱动车轮和机械臂，用于区分“摄像头/模型问题”和“车辆控制问题”。

### 10.3 电机串口

完整运行启动时应看到：

```text
[motor] CMD_INIT: got ACK, ESP32 is responding
```

如果显示 `NO REPLY`，说明程序已经打开串口，但 ESP32 没有正常应答，应检查：

1. 是否使用 `/dev/ttyS1`。
2. UART1 pinmux 是否显示配置成功。
3. ESP32 是否供电、是否与 SG2002 共地。
4. TX/RX 是否交叉连接。

### 10.4 机械臂不动作

先执行一条小范围、安全姿态测试：

```sh
./run.sh armpose ready --time 1000
```

如果日志显示已发送但机械臂无动作，检查 `/dev/ttyS2`、UART2 pinmux、舵机控制板
供电、共地和 TX/RX，而不是先修改抓球角度。

## 11. 开发机编译

目标架构为 `riscv64gc-unknown-linux-musl`，板端动态加载器为：

```text
/lib/ld-musl-riscv64v0p7_xthead.so.1
```

准备工具链和 TPU SDK 后执行：

```bash
cd akars-master
scripts/setup.sh
scripts/build.sh
```

也可以显式指定已有路径：

```bash
AKARS_TOOLCHAIN_DIR=/path/to/xuantie-v3.4.0 \
AKARS_TPU_SDK_DIR=/path/to/tpu-sdk-sg200x \
scripts/build.sh
```

输出文件为：

```text
target/riscv64gc-unknown-linux-musl/release/akars
```

板端运行还需要镜像中与模型匹配的 `libcviruntime.so`、`libcvikernel.so` 和
`libstdc++.so.6`。`run.sh` 已将 `/lib` 加入 `LD_LIBRARY_PATH`。

## 12. Starry开机自动运行

当前Starry镜像会在启动时自动执行可执行的 `/test_runner.sh`。部署时将
`scripts/starry-autostart.sh` 安装为该文件，并使用下面的标志文件控制是否启动：

```text
/root/akars-debug/autostart.enabled
```

启用后，Starry启动约1秒后自动运行 `/root/akars-debug/run.sh`。启动阶段失败时最多
重试3次；正常结束或人为停止后不会再次启动。运行日志输出到串口控制台，不持续写入SD卡。

停止当前程序并禁止下次自动运行：

```sh
pkill akars
mv /root/akars-debug/autostart.enabled \
   /root/akars-debug/autostart.disabled
```

重新启用：

```sh
mv /root/akars-debug/autostart.disabled \
   /root/akars-debug/autostart.enabled
```
