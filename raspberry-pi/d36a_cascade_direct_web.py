#!/usr/bin/env python3
"""Direct-GPIO controller for the D36A ball-and-beam axis.

Control structure:

    ball position error + filtered ball velocity -> PD stepper offset
    low-speed bias learning -> remove steady error caused by friction/tilt
    explicit stiction kick + velocity brake -> release a stuck steel ball
    stepper position worker -> STEP/DIR pulses

The web UI and runtime are locked to this single-loop controller so a stale
saved setting cannot silently switch the controller back to cascaded PID.

The process owns GPIO18/22/23 for the stepper, GPIO5/6/16/20 for four
active-low hardware buttons, and GPIO26 for the 3.3 V drive-state input.
Emergency stop keeps this process and the web UI
alive: it stops STEP, drives EN1 low, and preserves the current software step
count.  The operator may resume holding if the mechanism did not move, or
manually place the mechanism at its minimum and set a new zero.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import signal
import threading
import time
import urllib.request
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import lgpio
import serial


EN1 = 22
DIR1 = 23
STEP1 = 18
BUTTON_EMERGENCY = 5
BUTTON_REFERENCE_ZERO = 6
BUTTON_ENABLE_ZERO = 16
BUTTON_RUN_TRAJECTORY = 20
BUTTON_DEBOUNCE_S = 0.04
DRIVE_SIGNAL = 26
DRIVE_SIGNAL_DEBOUNCE_S = 0.015

DEFAULT_SETTINGS = {
    "target_cm": 0.0,
    "control_mode": 1,
    "pd_kp_steps_per_cm": 24.0,
    "pd_left_gain": 1.0,
    "pd_right_gain": 1.0,
    "pd_kd_steps_per_cm_s": 14.0,
    "bias_enabled": 1,
    "bias_ki_steps_per_cm_s": 8.0,
    "bias_limit_steps": 100.0,
    "bias_zone_cm": 6.0,
    "bias_velocity_limit_cm_s": 0.50,
    "pos_kp": 1.0,
    "pos_ki": 0.8,
    "pos_kd": 0.05,
    "pos_integral_limit": 30.0,
    "max_target_velocity_cm_s": 8.0,
    "deadband_cm": 0.10,
    "pos_integral_zone_cm": 1.50,
    "vel_kp": 5.0,
    "vel_ki": 1.0,
    "vel_kd": 0.0,
    "vel_integral_limit": 20.0,
    "stiction_ff_enabled": 0,
    "stiction_ff_positive_steps": 35.0,
    "stiction_ff_negative_steps": 150.0,
    "stiction_ff_fade_velocity_cm_s": 4.0,
    "max_offset_steps": 60,
    "control_center_steps": 235,
    "control_min_steps": 0,
    "control_max_steps": 681,
    "control_direction": 1,
    "velocity_alpha": 0.35,
    "control_hz": 25.0,
    "step_frequency_hz": 300.0,
    "vision_timeout_s": 0.30,
    "drive_ff_angle_deg": 3.0,
    "drive_ff_steps_per_degree": 10.0,
    "drive_ff_hold_s": 0.20,
    "drive_ff_fade_s": 0.35,
    "trajectory_level_steps": 314,
    "trajectory_right_push_delta_steps": 60,
    "trajectory_takeover_cm": 5.8,
    "trajectory_open_loop_timeout_s": 0.7,
    "trajectory_open_loop_frequency_hz": 1500.0,
    "escape_enabled": 1,
    "escape_velocity_threshold_cm_s": 0.15,
    "escape_error_threshold_cm": 0.60,
    "escape_stationary_s": 0.40,
    "escape_preload_steps": 100,
    "escape_preload_s": 0.40,
    "escape_direction": 1,
    "escape_start_steps": 40,
    "escape_ramp_steps_per_s": 120.0,
    "escape_push_steps": 160,
    "escape_release_velocity_cm_s": 0.30,
    "escape_release_displacement_cm": 0.15,
    "escape_release_confirm_frames": 3,
    "escape_push_max_s": 1.20,
    "escape_brake_min_steps": 30,
    "escape_brake_gain_steps_per_cm_s": 25.0,
    "escape_recovery_steps": 100,
    "escape_recovery_velocity_cm_s": 0.15,
    "escape_recovery_min_s": 0.15,
    "escape_recovery_max_s": 1.20,
    "escape_cooldown_s": 0.80,
}


HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>小球单环PD抗卡滞步进平衡</title>
  <style>
    :root { color-scheme:dark; font-family:"Microsoft YaHei",sans-serif; }
    * { box-sizing:border-box; }
    body { margin:0; background:#0d1117; color:#edf3f8; }
    main { width:min(98vw,1320px); margin:auto; padding:12px; display:flex; flex-direction:column; }
    h1 { margin:0 0 4px; font-size:22px; }
    h2 { margin:0 0 8px; font-size:16px; color:#58a6ff; }
    .subtitle { color:#9ba7b4; font-size:13px; margin-bottom:9px; }
    .viewer { overflow:hidden; border:2px solid #34404a; border-radius:10px; background:#000; }
    .viewer img { display:block; width:100%; min-height:120px; object-fit:contain; }
    .cards { display:grid; grid-template-columns:repeat(6,minmax(105px,1fr)); gap:7px; margin:9px 0; }
    .card,.panel { background:#161b22; border:1px solid #30363d; border-radius:9px; padding:10px; }
    .panel { margin-top:9px; }
    .removed-panel,.removed-field { display:none; }
    #vision-settings { order:100; }
    .label { color:#9ba7b4; font-size:12px; }
    .value { margin-top:3px; font:700 18px Consolas,monospace; }
    .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin-top:8px; }
    .grid { display:grid; grid-template-columns:repeat(4,minmax(145px,1fr)); gap:8px; }
    label.field { color:#9ba7b4; font-size:12px; }
    input,select,textarea { width:100%; margin-top:4px; background:#0d1117; color:#fff; border:1px solid #59636e; border-radius:6px; padding:8px; font-size:15px; }
    input.dirty,select.dirty { border-color:#d29922; box-shadow:0 0 0 1px #d29922; }
    button { border:0; border-radius:7px; padding:10px 15px; color:#fff; background:#238636; font-size:15px; cursor:pointer; }
    button.blue { background:#1f6feb; }
    button.secondary { background:#30363d; }
    button.warn { background:#9e6a03; }
    button.stop { background:#da3633; }
    button:disabled { opacity:.45; cursor:not-allowed; }
    .serial-param-card { background:#0d1117; border:1px solid #30363d; border-radius:8px; padding:8px; }
    .serial-param-head { display:flex; justify-content:space-between; align-items:center; color:#9ba7b4; font-size:12px; }
    .serial-param-head strong { color:#58a6ff; font:700 15px Consolas,monospace; }
    .serial-param-control { display:grid; grid-template-columns:44px 1fr 44px; gap:5px; align-items:end; }
    .serial-param-control input { min-width:0; text-align:center; font-family:Consolas,monospace; }
    .serial-param-control button { margin-top:4px; padding:8px 0; font-size:18px; }
    .serial-param-apply { width:100%; margin-top:5px; padding:6px; font-size:12px; }
    #notice { min-height:22px; margin-top:8px; }
    .ok { color:#3fb950; } .bad { color:#f85149; } .warn-text { color:#d29922; }
    .hint { color:#9ba7b4; font-size:13px; line-height:1.5; margin-top:7px; }
    .serial-log { min-height:150px; max-height:300px; overflow:auto; white-space:pre-wrap; background:#05080c; border:1px solid #30363d; border-radius:7px; padding:9px; font:13px Consolas,monospace; color:#b7f7c4; }
    @media(max-width:900px) {
      .cards { grid-template-columns:repeat(3,1fr); }
      .grid { grid-template-columns:repeat(2,1fr); }
    }
  </style>
</head>
<body>
<main>
  <h1>25 cm水管 · 单环PD抗卡滞步进控制</h1>
  <div class="subtitle">左端 -12.5 cm，中间 0，右端 +12.5 cm；右边界固定639 px，左边界X按步数0→235→681分段插值为40→14→0 px</div>
  <div class="viewer"><img id="vision" alt="实时二值化画面"></div>

  <div class="cards">
    <div class="card"><div class="label">控制状态</div><div class="value" id="mode">--</div></div>
    <div class="card"><div class="label">轴状态</div><div class="value" id="axisState">--</div></div>
    <div class="card"><div class="label">小球位置</div><div class="value" id="ball">N/A</div></div>
    <div class="card"><div class="label">目标位置</div><div class="value" id="target">0.00</div></div>
    <div class="card"><div class="label">位置误差</div><div class="value" id="posError">N/A</div></div>
    <div class="card"><div class="label">小球速度</div><div class="value" id="velocity">N/A</div></div>
    <div class="card"><div class="label">目标速度</div><div class="value" id="targetVelocity">0.00</div></div>
    <div class="card"><div class="label">速度误差</div><div class="value" id="velError">N/A</div></div>
    <div class="card"><div class="label">当前步数</div><div class="value" id="position">--</div></div>
    <div class="card"><div class="label">目标步数</div><div class="value" id="axisTarget">--</div></div>
    <div class="card"><div class="label">控制中心步数</div><div class="value" id="controlCenter">--</div></div>
    <div class="card"><div class="label">控制最大步数</div><div class="value" id="controlMax">--</div></div>
    <div class="card"><div class="label">脱困阶段</div><div class="value" id="escapeState">--</div></div>
    <div class="card"><div class="label">偏置学习</div><div class="value" id="integratorState">--</div></div>
    <div class="card"><div class="label">自学习偏置</div><div class="value" id="biasSteps">0</div></div>
    <div class="card"><div class="label">PD输出</div><div class="value" id="pdOutput">0</div></div>
    <div class="card"><div class="label">脱困/制动幅度</div><div class="value" id="escapeMagnitude">0</div></div>
    <div class="card"><div class="label">脱困次数</div><div class="value" id="escapeCount">0</div></div>
    <div class="card"><div class="label">视觉帧率</div><div class="value" id="fps">--</div></div>
    <div class="card"><div class="label">跟踪来源</div><div class="value" id="trackSource">--</div></div>
    <div class="card"><div class="label">本帧检测器</div><div class="value" id="measurementSource">--</div></div>
    <div class="card"><div class="label">跟踪置信度</div><div class="value" id="trackConfidence">--</div></div>
    <div class="card"><div class="label">一维投影强度</div><div class="value" id="projectionPeak">--</div></div>
    <div class="card"><div class="label">连续漏检帧</div><div class="value" id="missedFrames">--</div></div>
    <div class="card"><div class="label">插值左边界X</div><div class="value" id="pipeLeftX">--</div></div>
    <div class="card"><div class="label">步数同步</div><div class="value" id="axisSync">--</div></div>
    <div class="card"><div class="label">手动曝光</div><div class="value" id="exposureView">--</div></div>
    <div class="card"><div class="label">视觉裁剪</div><div class="value" id="cropView">--</div></div>
  </div>

  <div class="panel" id="vision-settings">
    <h2>视觉防抖、ROI与曝光</h2>
    <div class="grid">
      <label class="field">裁剪上边 y
        <input data-vision-key="crop_top" type="number" min="0" max="410" step="1">
      </label>
      <label class="field">裁剪下边 y
        <input data-vision-key="crop_bottom" type="number" min="70" max="480" step="1">
      </label>
      <label class="field">水管上边 y（裁剪内）
        <input data-vision-key="pipe_top_px" type="number" min="0" max="150" step="1">
      </label>
      <label class="field">水管下边 y（裁剪内）
        <input data-vision-key="pipe_bottom_px" type="number" min="1" max="180" step="1">
      </label>
      <label class="field">0步左边界 x
        <input data-vision-key="pipe_left_x_step0_px" type="number" min="0" max="638" step="1">
      </label>
      <label class="field">平衡步左边界 x
        <input data-vision-key="pipe_left_x_balance_px" type="number" min="0" max="638" step="1">
      </label>
      <label class="field">最高步左边界 x
        <input data-vision-key="pipe_left_x_max_px" type="number" min="0" max="638" step="1">
      </label>
      <label class="field">最低标定步数
        <input data-vision-key="pipe_min_steps" type="number" min="0" max="680" step="1">
      </label>
      <label class="field">平衡标定步数
        <input data-vision-key="pipe_balance_steps" type="number" min="1" max="680" step="1">
      </label>
      <label class="field">最高标定步数
        <input data-vision-key="pipe_max_steps" type="number" min="2" max="2000" step="1">
      </label>
      <label class="field">水管长度 cm
        <input data-vision-key="pipe_length_cm" type="number" min="1" max="100" step="0.1">
      </label>
      <label class="field">紫色识别框左边 cm
        <input data-vision-key="detect_min_cm" type="number" min="-12.5" max="12.4" step="0.5">
      </label>
      <label class="field">紫色识别框右边 cm
        <input data-vision-key="detect_max_cm" type="number" min="-12.4" max="12.5" step="0.5">
      </label>
      <label class="field">紫色识别框上边 y（裁剪内）
        <input data-vision-key="detect_top_px" type="number" min="0" max="150" step="1">
      </label>
      <label class="field">紫色识别框下边 y（裁剪内）
        <input data-vision-key="detect_bottom_px" type="number" min="1" max="180" step="1">
      </label>
      <label class="field">手动曝光（原自动约313）
        <input data-vision-key="exposure" type="number" min="1" max="5000" step="5">
      </label>
      <label class="field">光照归一化
        <select data-vision-key="illumination_normalization_enabled"><option value="1">开启</option><option value="0">关闭</option></select>
      </label>
      <label class="field">钢球局部对比阈值
        <input data-vision-key="contrast_drop" type="number" min="5" max="120" step="1">
      </label>
      <label class="field">局部背景核（奇数）
        <input data-vision-key="background_kernel" type="number" min="15" max="101" step="2">
      </label>
      <label class="field">钢球最小面积
        <input data-vision-key="min_area" type="number" min="5" max="5000" step="5">
      </label>
      <label class="field">钢球最大面积
        <input data-vision-key="max_area" type="number" min="10" max="10000" step="10">
      </label>
      <label class="field">水管掩膜内缩 px
        <input data-vision-key="pipe_roi_erode_px" type="number" min="1" max="31" step="2">
      </label>
      <label class="field">预测搜索半径 px
        <input data-vision-key="tracker_gate_px" type="number" min="10" max="250" step="5">
      </label>
      <label class="field">最大短时预测帧数
        <input data-vision-key="track_hold_frames" type="number" min="0" max="30" step="1">
      </label>
      <label class="field">Kalman过程噪声
        <input data-vision-key="kalman_process_noise" type="number" min="0.01" max="200" step="0.5">
      </label>
      <label class="field">Kalman测量噪声
        <input data-vision-key="kalman_measurement_noise" type="number" min="0.01" max="200" step="0.5">
      </label>
      <label class="field">一维备用检测
        <select data-vision-key="projection_enabled"><option value="1">开启</option><option value="0">关闭</option></select>
      </label>
      <label class="field">投影窗口宽度 px（奇数）
        <input data-vision-key="projection_window_px" type="number" min="9" max="61" step="2">
      </label>
      <label class="field">投影接受阈值
        <input data-vision-key="projection_threshold" type="number" min="50" max="2000" step="10">
      </label>
      <label class="field">水平边缘抑制百分位
        <input data-vision-key="projection_row_percentile" type="number" min="30" max="90" step="1">
      </label>
    </div>
    <div class="row">
      <button onclick="applyAll(false)">应用全部参数</button>
      <button class="secondary" onclick="applyAll(true)">保存全部参数并用于下次开机</button>
    </div>
    <div class="hint">
      不再视觉识别水管左右边界：右边界固定639 px，左边界X按0步=40、235步=14、681步=0分两段线性插值。
      紫色框的左、右、上、下四条边均可独立调节；青色框是完整水管坐标范围，调紫色框不会改变厘米坐标标定。
      绿色框是本帧轮廓候选，红色十字是实际用于控制的坐标，两者在真实检测帧应重合；黄色十字仅表示短时预测。
      二维轮廓破碎或钢球贴住画面端点时，会自动使用按行消除水管横边的一维灰度投影继续定位。
    </div>
  </div>

  <div class="panel">
    <h2>目标与闭环</h2>
    <div class="row">
      <button class="blue" onclick="setTarget(-5)">目标 -5 cm</button>
      <button class="blue" onclick="setTarget(0)">目标 0 cm</button>
      <button class="blue" onclick="setTarget(5)">目标 +5 cm</button>
      <button onclick="action('enable')">开启闭环</button>
      <button class="warn" onclick="action('disable')">停止闭环并保持</button>
      <button class="secondary" onclick="action('return_balance')">回控制中心</button>
      <button class="warn" onclick="testEscape()">测试一次大角度脱困</button>
      <button class="stop" onclick="emergencyStop()">紧急停止并释放</button>
    </div>
    <div id="notice" class="warn-text">新轴首次启动必须人工放到最低位置，再确认零点。</div>
  </div>

  <div class="panel">
    <h2>轴恢复与手动控制</h2>
    <div class="row">
      <button class="warn" onclick="referenceZero()">确认当前位置为最低点0</button>
      <button class="secondary" onclick="action('resume_axis')">急停后恢复轴保持</button>
      <button class="blue" onclick="move(-10)">-10</button>
      <button class="blue" onclick="move(-1)">-1</button>
      <button onclick="move(1)">+1</button>
      <button onclick="move(10)">+10</button>
      <input id="gotoValue" type="number" min="0" max="681" step="1" value="235" style="width:110px">
      <button class="secondary" onclick="gotoPosition()">移动到绝对步数</button>
    </div>
    <div class="hint">
      急停不会关闭网页或删除轴对象。若机构急停后没有被手动移动，可点“恢复轴保持”；
      若机构位置发生变化，请人工放回最低点，再点“确认当前位置为最低点0”。
    </div>
  </div>

  <div class="panel">
    <h2>树莓派实体按键（低电平有效）</h2>
    <div class="row">
      <span>BCM5 急停：<strong id="buttonEmergency">--</strong></span>
      <span>BCM6 最低点0：<strong id="buttonReference">--</strong></span>
      <span>BCM16 0点闭环：<strong id="buttonZeroLoop">--</strong></span>
      <span>BCM20 运行本题：<strong id="buttonTrajectory">--</strong></span>
      <span>触发次数：<strong id="buttonCount">0</strong></span>
    </div>
    <div class="hint">
      每个按钮的一端接对应BCM GPIO，另一端接任意GND；程序已启用树莓派内部上拉。
      显示“松开(1)”为正常，按下时显示“按下(0)”。最后动作：<strong id="buttonLastAction">待机</strong>
    </div>
  </div>

  <div class="panel" id="arbitrary-target">
    <h2>整根水管任意目标位置</h2>
    <div class="row">
      <input id="arbitraryTargetNumber" type="number" min="-12.5" max="12.5" step="0.1" value="0" style="width:110px" oninput="syncArbitraryTarget('number')">
      <input id="arbitraryTargetRange" type="range" min="-12.5" max="12.5" step="0.1" value="0" style="min-width:320px" oninput="syncArbitraryTarget('range')">
      <button class="blue" onclick="setArbitraryTarget()">设为目标并开启闭环</button>
      <button class="secondary" onclick="setArbitraryPreset(-10)">-10 cm</button>
      <button class="secondary" onclick="setArbitraryPreset(0)">0 cm</button>
      <button class="secondary" onclick="setArbitraryPreset(10)">+10 cm</button>
      <span>准备设定：<strong id="arbitraryTargetReadout">0.0 cm</strong></span>
    </div>
    <div class="hint">
      可直接输入或拖动选择−12.5～+12.5 cm的任意位置；点击后立即写入目标并开启现有单环PD闭环。
      靠近管端时机械余量和视觉余量更小，建议首次测试先使用−10～+10 cm范围。
    </div>
  </div>

  <div class="panel" id="drive-feedforward">
    <h2>小车发车/停车前馈角度</h2>
    <div class="grid">
      <label class="field">前馈角度绝对值 °
        <input data-key="drive_ff_angle_deg" type="number" min="0" max="90" step="0.1">
      </label>
      <label class="field">机构换算 步/°
        <input data-key="drive_ff_steps_per_degree" type="number" min="0" max="681" step="0.5">
      </label>
      <label class="field">满角保持时间 s
        <input data-key="drive_ff_hold_s" type="number" min="0" max="30" step="0.05">
      </label>
      <label class="field">线性回零时间 s
        <input data-key="drive_ff_fade_s" type="number" min="0.01" max="30" step="0.05">
      </label>
    </div>
    <div class="row">
      <button onclick="applyAll(false)">应用前馈参数</button>
      <button class="secondary" onclick="applyAll(true)">保存前馈参数</button>
      <button class="blue" onclick="testDriveFeedforward(1)">测试向右发车前馈</button>
      <button class="warn" onclick="testDriveFeedforward(0)">测试向左停车前馈</button>
      <span>BCM26输入：<strong id="driveSignalLevel">低(0) 停车</strong></span>
      <span>最近边沿：<strong id="driveSignalEdge">待机</strong></span>
      <span>前馈阶段：<strong id="driveFeedforwardPhase">待机</strong></span>
      <span>当前角度：<strong id="driveFeedforwardAngle">0.00°</strong></span>
      <span>当前补偿：<strong id="driveFeedforwardSteps">0步</strong></span>
      <span>边沿次数：<strong id="driveSignalCount">0</strong></span>
    </div>
    <div class="hint">
      BCM26（物理37）只接受3.3 V逻辑信号并使用内部下拉：上升沿触发向右正角度，下降沿触发向左负角度；
      满角保持后线性衰减为0并交回位置闭环。默认3°×10步/°=30步；当前机构按增加步数为向右、减少步数为向左。
      上游若是5 V或12 V信号，必须先经过电平转换、分压或光耦，严禁直接连接树莓派。
    </div>
  </div>

  <div class="panel" id="stm32-serial">
    <h2>STM32 USART1 参数调试</h2>
    <div class="row">
      <span>接口：<strong>/dev/ttyAMA4</strong></span>
      <span>参数：<strong>115200 8N1</strong></span>
      <span>状态：<strong id="serialState">未连接</strong></span>
      <span>RX：<strong id="serialRxCount">0 B / 0行</strong></span>
      <span>TX：<strong id="serialTxCount">0 B / 0次</strong></span>
      <span>已解析：<strong id="serialParsedCount">0项</strong></span>
      <span>最近接收：<strong id="serialLastRx">--</strong></span>
    </div>
    <div class="row">
      <button class="blue" onclick="sendSerialQuick('GET')">一键读取全部参数</button>
      <label>加减倍率
        <select id="serialStepMultiplier" style="width:auto;margin:0 0 0 5px">
          <option value="1">×1 精细</option>
          <option value="10">×10</option>
          <option value="100">×100</option>
        </select>
      </label>
      <button class="blue" onclick="sendAllSerialParameters()">应用全部参数</button>
      <button class="secondary" onclick="sendSerialQuick('SAVE')">保存到STM32</button>
      <button class="secondary" onclick="sendSerialQuick('LOAD')">重新载入</button>
      <button class="secondary" onclick="sendSerialQuick('STATUS')">读取状态</button>
      <button class="warn" onclick="sendSerialQuick('DEFAULT')">恢复默认</button>
    </div>
    <div class="grid" id="serialParameterGrid">
      <div class="serial-param-card"><div class="serial-param-head"><strong>VKP</strong><span>步长0.01</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('VKP',-1)">−</button><input data-serial-param="VKP" data-serial-step="0.01" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('VKP',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('VKP')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>VKI</strong><span>步长0.01</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('VKI',-1)">−</button><input data-serial-param="VKI" data-serial-step="0.01" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('VKI',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('VKI')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>TKP</strong><span>步长0.01</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('TKP',-1)">−</button><input data-serial-param="TKP" data-serial-step="0.01" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('TKP',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('TKP')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>TKD</strong><span>步长0.01</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('TKD',-1)">−</button><input data-serial-param="TKD" data-serial-step="0.01" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('TKD',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('TKD')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>P1_MAX</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('P1_MAX',-1)">−</button><input data-serial-param="P1_MAX" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('P1_MAX',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('P1_MAX')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>P2_MAX</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('P2_MAX',-1)">−</button><input data-serial-param="P2_MAX" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('P2_MAX',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('P2_MAX')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>AB_MAX</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('AB_MAX',-1)">−</button><input data-serial-param="AB_MAX" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('AB_MAX',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('AB_MAX')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>P1_STOP</strong><span>步长100</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('P1_STOP',-1)">−</button><input data-serial-param="P1_STOP" data-serial-step="100" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('P1_STOP',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('P1_STOP')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>P2_STOP</strong><span>步长100</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('P2_STOP',-1)">−</button><input data-serial-param="P2_STOP" data-serial-step="100" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('P2_STOP',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('P2_STOP')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>AB_STOP</strong><span>步长100</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('AB_STOP',-1)">−</button><input data-serial-param="AB_STOP" data-serial-step="100" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('AB_STOP',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('AB_STOP')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>CENTER</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('CENTER',-1)">−</button><input data-serial-param="CENTER" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('CENTER',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('CENTER')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>DEAD</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('DEAD',-1)">−</button><input data-serial-param="DEAD" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('DEAD',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('DEAD')">应用当前值</button></div>
      <div class="serial-param-card"><div class="serial-param-head"><strong>MAX_TURN</strong><span>步长1</span></div><div class="serial-param-control"><button type="button" class="secondary" onclick="adjustSerialParameter('MAX_TURN',-1)">−</button><input data-serial-param="MAX_TURN" data-serial-step="1" type="number" step="any" placeholder="未读取"><button type="button" class="blue" onclick="adjustSerialParameter('MAX_TURN',1)">+</button></div><button type="button" class="serial-param-apply" onclick="sendSerialParameter('MAX_TURN')">应用当前值</button></div>
    </div>
    <div class="row">
      <span class="hint">点击“− / +”会按当前倍率修改并立即发送；也可以直接改数字后点“应用当前值”或按Enter。修改后点“保存到STM32”才会掉电保存。</span>
    </div>
    <div class="row"><span id="serialError" class="warn-text"></span></div>
    <details>
      <summary>高级：查看串口日志或发送原始命令</summary>
      <div class="row"><textarea id="serialTxData" rows="2" placeholder="原始命令，例如：SET,TKP,0.60"></textarea></div>
      <div class="row">
        <label style="display:flex;align-items:center;gap:6px"><input id="serialAppendNewline" type="checkbox" checked style="width:auto;margin:0">自动追加换行 \\n</label>
        <button class="blue" onclick="sendSerialData()">发送原始命令</button>
        <button class="secondary" onclick="sendSerialQuick('HELP')">读取HELP</button>
        <button class="secondary" onclick="clearSerialLog()">清空日志</button>
      </div>
      <pre id="serialLog" class="serial-log">等待串口数据…</pre>
    </details>
    <div class="hint">
      树莓派物理24 / GPIO8 / UART4_TX接STM32 PA10；物理21 / GPIO9 / UART4_RX接STM32 PA9；两端GND必须共地。
      接收内容优先按换行分帧；没有换行时，串口空闲200 ms也会自动显示为一帧。支持从 PARAM=value、PARAM:value、PARAM,value 或 PARAM value 格式自动提取参数。
    </div>
  </div>

  <div class="panel" id="trajectory-test">
    <h2>独立题目：O → +5 cm → -5 cm（开环后闭环）</h2>
    <div class="grid">
      <label class="field">一键实测平衡轴位 步
        <input data-key="trajectory_level_steps" data-trajectory-key type="number" min="0" max="681" step="1">
      </label>
      <label class="field">向右小角度增量 步
        <input data-key="trajectory_right_push_delta_steps" data-trajectory-key type="number" min="1" max="200" step="1">
      </label>
      <label class="field">第一阶段开环STEP频率 Hz
        <input data-key="trajectory_open_loop_frequency_hz" data-trajectory-key type="number" min="20" max="5000" step="10">
      </label>
      <label class="field">正向超程保护位置 cm
        <input data-key="trajectory_takeover_cm" data-trajectory-key type="number" min="4" max="6" step="0.1">
      </label>
      <label class="field">固定开环时间 s
        <input data-key="trajectory_open_loop_timeout_s" data-trajectory-key type="number" min="0.2" max="4" step="0.1">
      </label>
    </div>
    <div class="row">
      <button onclick="applyTrajectory(false)">应用第一阶段参数</button>
      <button class="secondary" onclick="applyTrajectory(true)">保存第一阶段参数</button>
      <button id="trajectoryStart" class="blue" onclick="startTrajectory()">启动本题（开环→闭环）</button>
      <button id="trajectoryStop" class="stop" onclick="stopTrajectory()">停止并保持当前位置</button>
      <span>阶段：<strong id="trajectoryPhase">待机</strong></span>
      <span>正式计时：<strong id="trajectoryElapsed">0.00 s</strong></span>
      <span>开环推杆：<strong id="trajectoryOpenLoopSteps">--</strong></span>
      <span>接管原因：<strong id="trajectoryTakeoverReason">--</strong></span>
      <span>+5峰值：<strong id="trajectoryPlusPeak">--</strong></span>
      <span>-5峰值：<strong id="trajectoryMinusPeak">--</strong></span>
      <span>最终误差：<strong id="trajectoryFinalError">--</strong></span>
      <span>结果：<strong id="trajectoryResult">尚未运行</strong></span>
    </div>
    <div class="hint">
      本题是独立过程：先把小球放在 O 点附近，点击控制键后不再等待普通PID稳定，也不执行预备回位；
      程序立即关闭普通PID，并直接命令步进轴从标定平衡位314步开环推到“314+向右增量”，固定时间到后启用目标-5 cm的单环PD。
      若小球提前到达正向超程保护位置则立即切换闭环，防止超过+6 cm；开环结束后以314步作为本题闭环中心。
      第一阶段参数可单独应用或保存，不会启动电机；“向右小角度增量”是相对步数，不再使用533之类的绝对大角度目标。
      正式时长固定5秒，要求正负峰值、最终位置以及最后0.5秒均在目标±1 cm内。机械0～681步软限位仍保留。
      一键轨迹已取消原来的钢球±8 cm越界中止，钢球到水管两端时也会继续执行；视觉丢失和机械步数限位仍保留。
    </div>
  </div>

  <div class="panel removed-panel" id="shake-test">
    <h2>步进电机抖动频率测试</h2>
    <div class="grid">
      <label class="field">测试模式
        <select id="shakeMode">
          <option value="fixed">固定STEP频率</option>
          <option value="sweep">自动逐档扫频</option>
        </select>
      </label>
      <label class="field">固定STEP频率 Hz
        <input id="shakeFrequency" type="number" min="20" max="5000" step="10" value="450">
      </label>
      <label class="field">单边抖动幅度 步
        <input id="shakeAmplitude" type="number" min="1" max="300" step="1" value="20">
      </label>
      <label class="field">固定频率持续时间 s
        <input id="shakeDuration" type="number" min="0.5" max="30" step="0.5" value="5">
      </label>
      <label class="field">扫频起点 Hz
        <input id="sweepStart" type="number" min="20" max="5000" step="10" value="200">
      </label>
      <label class="field">扫频终点 Hz
        <input id="sweepEnd" type="number" min="20" max="5000" step="10" value="3000">
      </label>
      <label class="field">每档停留时间 s
        <input id="sweepDwell" type="number" min="0.3" max="10" step="0.1" value="1.5">
      </label>
      <label class="field">理论完整往返频率
        <input id="shakeEstimate" type="text" value="--" readonly>
      </label>
    </div>
    <div class="row">
      <button class="secondary" onclick="setShakeFrequency(200)">200 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(450)">450 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(800)">800 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(1200)">1200 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(2000)">2000 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(3000)">3000 Hz</button>
      <button class="secondary" onclick="setShakeFrequency(5000)">5000 Hz</button>
    </div>
    <div class="row">
      <button id="shakeStart" class="warn" onclick="startShake()">开始抖动测试</button>
      <button id="shakeStop" class="stop" onclick="stopShake()">停止测试并回中心</button>
      <span>状态：<strong id="shakeState">待机</strong></span>
      <span>当前指令：<strong id="shakeCommand">--</strong></span>
      <span>程序实发步频：<strong id="shakeActualStep">--</strong></span>
      <span>程序实发往返：<strong id="shakeActualCycle">--</strong></span>
      <span>完成周期：<strong id="shakeCycles">0</strong></span>
    </div>
    <div class="hint">
      测试会自动暂停位置闭环，以启动瞬间的位置为中心在±幅度内往返，并始终受0～681软限位保护；结束后回到测试中心，闭环保持关闭。
      STEP脉冲频率是指令移动速度，不是水管完整往返频率；理论往返频率约为 STEP频率÷(4×单边幅度)。“程序实发”值由GPIO脉冲和软件步数计算，只能反映树莓派发脉冲的能力；当前没有读取电机编码器，无法自动判断转子失步。
      建议先用±20步从200 Hz逐档上升，出现失步、异响或机构跟不上时立即停止，不能把5000 Hz直接当作可靠速度。
    </div>
  </div>

  <div class="panel">
    <h2>单环PD＋低速静差补偿</h2>
    <div class="grid">
      <label class="field">目标位置 cm
        <input data-key="target_cm" type="number" min="-12.5" max="12.5" step="0.1">
      </label>
      <label class="field">单环Kp 步/cm
        <input data-key="pd_kp_steps_per_cm" type="number" min="0" max="1000" step="1">
      </label>
      <label class="field">向左比例力度系数
        <input data-key="pd_left_gain" type="number" min="0" max="5" step="0.05">
      </label>
      <label class="field">向右比例力度系数
        <input data-key="pd_right_gain" type="number" min="0" max="5" step="0.05">
      </label>
      <label class="field">速度阻尼Kd 步/(cm/s)
        <input data-key="pd_kd_steps_per_cm_s" type="number" min="0" max="1000" step="1">
      </label>
      <label class="field">自动学习平衡偏置
        <select data-key="bias_enabled"><option value="1">开启</option><option value="0">关闭</option></select>
      </label>
      <label class="field">偏置学习速度 步/(cm·s)
        <input data-key="bias_ki_steps_per_cm_s" type="number" min="0" max="100" step="0.1">
      </label>
      <label class="field">偏置最大幅度 步
        <input data-key="bias_limit_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field">偏置学习作用区 cm
        <input data-key="bias_zone_cm" type="number" min="0" max="12.5" step="0.1">
      </label>
      <label class="field">学习时最大球速 cm/s
        <input data-key="bias_velocity_limit_cm_s" type="number" min="0" max="20" step="0.05">
      </label>
    </div>
    <div class="row">
      <button onclick="applyAll(false)">应用全部参数</button>
      <button class="secondary" onclick="applyAll(true)">保存全部参数</button>
    </div>
    <div class="hint">
      输出 = Kp×位置误差×左右力度系数 − Kd×球速 ＋ 有界静差补偿。左系数作用于负方向位置误差，右系数作用于正方向位置误差；
      两者只缩放比例力度，不改变速度阻尼、静差补偿、脱困和现有限位。只要误差位于“偏置学习作用区”且球速较低，补偿就会继续修正倾角；
      钢球开始运动时自动冻结，接近补偿上限后才允许转入脱困。目标+5却停在+2时，应保证作用区≥3 cm并适当增大偏置上限。
    </div>
  </div>

  <div class="panel removed-panel">
    <h2>回退模式外环：位置PID → 目标速度</h2>
    <div class="grid">
      <label class="field">目标位置 cm
        <input type="number" min="-12.5" max="12.5" step="0.1">
      </label>
      <label class="field">位置Kp
        <input data-key="pos_kp" type="number" min="0" step="0.05">
      </label>
      <label class="field">位置Ki（消除静差）
        <input data-key="pos_ki" type="number" min="0" step="0.05">
      </label>
      <label class="field">位置Kd（速度阻尼）
        <input data-key="pos_kd" type="number" min="0" step="0.01">
      </label>
      <label class="field">位置积分限幅
        <input data-key="pos_integral_limit" type="number" min="0" step="1">
      </label>
      <label class="field">最大目标速度 cm/s
        <input data-key="max_target_velocity_cm_s" type="number" min="0" step="0.2">
      </label>
      <label class="field">位置死区 cm
        <input data-key="deadband_cm" type="number" min="0" max="12.5" step="0.05">
      </label>
      <label class="field">位置积分作用区 cm
        <input data-key="pos_integral_zone_cm" type="number" min="0" max="12.5" step="0.1">
      </label>
    </div>
  </div>

  <div class="panel">
    <h2>执行器、控制频率与通用限位</h2>
    <div class="grid">
      <label class="field removed-field">速度Kp
        <input data-key="vel_kp" type="number" min="0" step="0.1">
      </label>
      <label class="field removed-field">速度Ki（克服机构偏置）
        <input data-key="vel_ki" type="number" min="0" step="0.1">
      </label>
      <label class="field removed-field">速度Kd
        <input data-key="vel_kd" type="number" min="0" step="0.05">
      </label>
      <label class="field removed-field">速度积分限幅
        <input data-key="vel_integral_limit" type="number" min="0" step="1">
      </label>
      <label class="field removed-field">串级静摩擦前馈
        <select data-key="stiction_ff_enabled"><option value="1">开启</option><option value="0">关闭</option></select>
      </label>
      <label class="field removed-field">向正方向起滑补偿 步
        <input data-key="stiction_ff_positive_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field removed-field">向负方向起滑补偿 步
        <input data-key="stiction_ff_negative_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field removed-field">补偿淡出球速 cm/s
        <input data-key="stiction_ff_fade_velocity_cm_s" type="number" min="0.05" max="50" step="0.1">
      </label>
      <label class="field">最大控制偏移步数
        <input data-key="max_offset_steps" type="number" min="1" max="681" step="1">
      </label>
      <label class="field">控制中心步数（水管实际水平）
        <input data-key="control_center_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field">控制最小步数
        <input data-key="control_min_steps" type="number" min="0" max="680" step="1">
      </label>
      <label class="field">控制最大步数
        <input data-key="control_max_steps" type="number" min="1" max="681" step="1">
      </label>
      <label class="field">控制方向
        <select data-key="control_direction"><option value="1">+1</option><option value="-1">-1</option></select>
      </label>
      <label class="field">速度滤波系数
        <input data-key="velocity_alpha" type="number" min="0.001" max="1" step="0.01">
      </label>
      <label class="field">控制频率 Hz
        <input data-key="control_hz" type="number" min="1" max="100" step="1">
      </label>
      <label class="field">步进脉冲频率 Hz
        <input data-key="step_frequency_hz" type="number" min="1" max="5000" step="10">
      </label>
      <label class="field">视觉丢失超时 s
        <input data-key="vision_timeout_s" type="number" min="0.01" max="30" step="0.05">
      </label>
    </div>
    <div class="row">
      <button onclick="applyAll(false)">应用全部参数</button>
      <button class="secondary" onclick="applyAll(true)">保存全部参数并用于下次开机</button>
    </div>
    <div class="hint">
      控制中心可用于补偿水管安装不水平；最大偏移范围已放宽到681步，但最终目标仍受控制最小/最大步数限制。静摩擦前馈只在目标速度仍指向位置目标时加入，并随钢球沿目标方向加速而淡出；正、负方向可分别设置，避免靠I项蓄能破静摩擦。
    </div>
  </div>

  <div class="panel">
    <h2>卡滞检测、自适应破静摩擦与速度制动</h2>
    <div class="grid">
      <label class="field">自动脱困
        <select data-key="escape_enabled"><option value="1">开启</option><option value="0">关闭</option></select>
      </label>
      <label class="field">静止速度阈值 cm/s
        <input data-key="escape_velocity_threshold_cm_s" type="number" min="0" max="100" step="0.01">
      </label>
      <label class="field">最小目标误差 cm
        <input data-key="escape_error_threshold_cm" type="number" min="0" max="12.5" step="0.1">
      </label>
      <label class="field">判定静止时间 s
        <input data-key="escape_stationary_s" type="number" min="0.01" max="60" step="0.05">
      </label>
      <label class="field">脱困预加载幅度 步
        <input data-key="escape_preload_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field">脱困预加载时间 s
        <input data-key="escape_preload_s" type="number" min="0" max="5" step="0.05">
      </label>
      <label class="field">脱困方向
        <select data-key="escape_direction"><option value="1">+1（当前机构）</option><option value="-1">-1</option></select>
      </label>
      <label class="field">破静摩擦起始幅度 步
        <input data-key="escape_start_steps" type="number" min="1" max="681" step="1">
      </label>
      <label class="field">破静摩擦递增速度 步/s
        <input data-key="escape_ramp_steps_per_s" type="number" min="0" max="5000" step="10">
      </label>
      <label class="field">破静摩擦最大幅度 步
        <input data-key="escape_push_steps" type="number" min="1" max="681" step="1">
      </label>
      <label class="field">启动检测速度 cm/s
        <input data-key="escape_release_velocity_cm_s" type="number" min="0" max="100" step="0.05">
      </label>
      <label class="field">启动检测位移 cm
        <input data-key="escape_release_displacement_cm" type="number" min="0" max="12.5" step="0.05">
      </label>
      <label class="field">启动确认连续帧
        <input data-key="escape_release_confirm_frames" type="number" min="1" max="30" step="1">
      </label>
      <label class="field">最大推离时间 s
        <input data-key="escape_push_max_s" type="number" min="0.01" max="30" step="0.05">
      </label>
      <label class="field">初始制动保底幅度 步
        <input data-key="escape_brake_min_steps" type="number" min="0" max="681" step="1">
      </label>
      <label class="field">速度制动增益 步/(cm/s)
        <input data-key="escape_brake_gain_steps_per_cm_s" type="number" min="0" max="1000" step="1">
      </label>
      <label class="field">最大制动幅度 步
        <input data-key="escape_recovery_steps" type="number" min="1" max="681" step="1">
      </label>
      <label class="field">制动退出速度 cm/s
        <input data-key="escape_recovery_velocity_cm_s" type="number" min="0" max="100" step="0.05">
      </label>
      <label class="field">最短制动时间 s
        <input data-key="escape_recovery_min_s" type="number" min="0" max="30" step="0.05">
      </label>
      <label class="field">最长制动时间 s
        <input data-key="escape_recovery_max_s" type="number" min="0.01" max="30" step="0.05">
      </label>
      <label class="field">再次触发冷却 s
        <input data-key="escape_cooldown_s" type="number" min="0" max="120" step="0.1">
      </label>
    </div>
    <div class="row">
      <button onclick="applyAll(false)">应用全部参数</button>
      <button class="secondary" onclick="applyAll(true)">保存全部参数并用于下次开机</button>
      <button class="warn" onclick="testEscape()">测试一次脱困</button>
    </div>
    <div class="hint">
      卡滞条件一出现即冻结偏置学习（回退模式则冻结两级积分）；静止达到判定时间后，从起始幅度逐渐增大到最大幅度。
      小球一旦产生朝目标的有效速度或位移，立即停止推离，按实时速度比例反向制动；制动结束后交回所选闭环算法。
    </div>
  </div>
</main>
<script>
document.getElementById("vision").src =
  location.protocol + "//" + location.hostname + ":8080/stream.mjpg";
const visionBase = location.protocol + "//" + location.hostname + ":8080";
const controlFields = [...document.querySelectorAll("[data-key]")];
const visionFields = [...document.querySelectorAll("[data-vision-key]")];
let controlInitialized = false;
let visionInitialized = false;
[...controlFields,...visionFields].forEach(
  item => item.addEventListener("input", () => item.classList.add("dirty"))
);

function fmt(value,digits=2,suffix="") {
  return value === null || value === undefined ? "N/A" : Number(value).toFixed(digits)+suffix;
}
function notice(message,kind="ok") {
  const item=document.getElementById("notice"); item.textContent=message; item.className=kind;
}
async function post(payload) {
  const response=await fetch("/control",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(payload)});
  const data=await response.json();
  if(!response.ok) throw new Error(data.error||"操作失败");
  notice(data.message||"操作完成"); setTimeout(refresh,120);
}
async function postVision(payload) {
  const response=await fetch(visionBase+"/settings",{
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(payload)
  });
  const data=await response.json();
  if(!response.ok) throw new Error(data.error||"视觉参数操作失败");
  return data;
}
function action(name){post({action:name}).catch(error=>notice(error.message,"bad"));}
function setTarget(value){post({action:"set_target",target_cm:value,enable:true}).catch(error=>notice(error.message,"bad"));}
function move(steps){post({action:"move",steps}).catch(error=>notice(error.message,"bad"));}
function gotoPosition(){post({action:"goto",position:Number(document.getElementById("gotoValue").value)}).catch(error=>notice(error.message,"bad"));}
function referenceZero(){if(confirm("确认机构已经人工放到安全最低位置？")) action("reference_zero");}
function emergencyStop(){if(confirm("立即停STEP并释放EN？网页会保留，可再次恢复。")) action("emergency_stop");}
function testEscape(){if(confirm("按当前脱困角度朝目标大幅摆动，并在小球运动后反向补偿？机构会产生明显运动。")) action("test_escape");}
function setShakeFrequency(value){
  document.getElementById("shakeMode").value="fixed";
  document.getElementById("shakeFrequency").value=value;
  updateShakeEstimate();
}
function updateShakeEstimate(){
  const frequency=Number(document.getElementById("shakeFrequency").value);
  const amplitude=Number(document.getElementById("shakeAmplitude").value);
  document.getElementById("shakeEstimate").value=
    frequency>0&&amplitude>0?(frequency/(4*amplitude)).toFixed(2)+" Hz":"--";
}
async function startShake(){
  const mode=document.getElementById("shakeMode").value;
  const payload={
    action:"shake_start", mode,
    frequency_hz:Number(document.getElementById("shakeFrequency").value),
    amplitude_steps:Number(document.getElementById("shakeAmplitude").value),
    duration_s:Number(document.getElementById("shakeDuration").value),
    sweep_start_hz:Number(document.getElementById("sweepStart").value),
    sweep_end_hz:Number(document.getElementById("sweepEnd").value),
    sweep_dwell_s:Number(document.getElementById("sweepDwell").value)
  };
  const description=mode==="sweep"
    ?`${payload.sweep_start_hz}～${payload.sweep_end_hz} Hz逐档扫频`
    :`${payload.frequency_hz} Hz固定频率`;
  if(!confirm(`将暂停闭环并以当前位置为中心进行±${payload.amplitude_steps}步、${description}测试，确认机构有安全活动空间？`))return;
  post(payload).catch(error=>notice(error.message,"bad"));
}
function stopShake(){post({action:"shake_stop"}).catch(error=>notice(error.message,"bad"));}
function startTrajectory(){
  post({action:"trajectory_start"}).catch(error=>notice(error.message,"bad"));
}
function stopTrajectory(){
  post({action:"trajectory_stop"}).catch(error=>notice(error.message,"bad"));
}
function syncArbitraryTarget(source){
  const number=document.getElementById("arbitraryTargetNumber");
  const range=document.getElementById("arbitraryTargetRange");
  let value=Number(source==="range"?range.value:number.value);
  value=Math.max(-12.5,Math.min(12.5,value));
  number.value=value.toFixed(1); range.value=value;
  document.getElementById("arbitraryTargetReadout").textContent=value.toFixed(1)+" cm";
}
function setArbitraryTarget(){
  const value=Number(document.getElementById("arbitraryTargetNumber").value);
  post({action:"set_target",target_cm:value,enable:true}).catch(error=>notice(error.message,"bad"));
}
function setArbitraryPreset(value){
  document.getElementById("arbitraryTargetNumber").value=value;
  syncArbitraryTarget("number"); setArbitraryTarget();
}
function testDriveFeedforward(level){
  post({action:"drive_feedforward_test",level}).catch(error=>notice(error.message,"bad"));
}
const SERIAL_PARAMETERS=[
  "VKP","VKI","TKP","TKD",
  "P1_MAX","P2_MAX","AB_MAX",
  "P1_STOP","P2_STOP","AB_STOP",
  "CENTER","DEAD","MAX_TURN"
];
function serialParameterInput(name){
  return document.querySelector(`[data-serial-param="${name}"]`);
}
let serialCommandQueue=Promise.resolve();
const serialParameterHoldUntil={};
async function sendSerialCommand(command){
  const operation=serialCommandQueue.then(
    ()=>post({action:"serial_send",data:command,append_newline:true})
  );
  serialCommandQueue=operation.catch(()=>{});
  await operation;
}
async function sendSerialQuick(command){
  try{
    if(command==="DEFAULT"&&!confirm("确认让STM32恢复默认参数？如需永久保存，恢复后还要再点SAVE。"))return;
    const data=command==="LOAD"||command==="DEFAULT"?`${command}\\nGET`:command;
    await sendSerialCommand(data);
  }catch(error){notice(error.message,"bad");}
}
async function sendSerialParameter(name){
  try{
    if(!SERIAL_PARAMETERS.includes(name))throw new Error("不支持的STM32参数名");
    const input=serialParameterInput(name);
    const value=input.value.trim();
    if(value===""||!Number.isFinite(Number(value)))throw new Error(`${name}请输入有效数值`);
    serialParameterHoldUntil[name]=Date.now()+1200;
    await sendSerialCommand(`SET,${name},${value}\\nGET`);
  }catch(error){notice(error.message,"bad");}
}
async function adjustSerialParameter(name,direction){
  try{
    const input=serialParameterInput(name);
    const current=Number(input.value);
    if(input.value.trim()===""||!Number.isFinite(current)){
      throw new Error("请先点击“一键读取全部参数”");
    }
    const baseStep=Number(input.dataset.serialStep);
    const multiplier=Number(document.getElementById("serialStepMultiplier").value);
    const next=Math.round((current+direction*baseStep*multiplier)*1000000)/1000000;
    input.value=String(next);
    await sendSerialParameter(name);
  }catch(error){notice(error.message,"bad");}
}
async function sendAllSerialParameters(){
  try{
    const commands=[];
    for(const name of SERIAL_PARAMETERS){
      const value=serialParameterInput(name).value.trim();
      if(value==="")continue;
      if(!Number.isFinite(Number(value)))throw new Error(`${name}请输入有效数值`);
      serialParameterHoldUntil[name]=Date.now()+1200;
      commands.push(`SET,${name},${value}`);
    }
    if(!commands.length)throw new Error("请至少填写一个STM32参数");
    commands.push("GET");
    await sendSerialCommand(commands.join("\\n"));
  }catch(error){notice(error.message,"bad");}
}
async function sendSerialData(){
  try{
    const data=document.getElementById("serialTxData").value;
    const appendNewline=document.getElementById("serialAppendNewline").checked;
    await post({action:"serial_send",data,append_newline:appendNewline});
  }catch(error){notice(error.message,"bad");}
}
function clearSerialLog(){
  post({action:"serial_clear"}).catch(error=>notice(error.message,"bad"));
}
document.querySelectorAll("[data-serial-param]").forEach(input=>{
  input.addEventListener("keydown",event=>{
    if(event.key==="Enter"){
      event.preventDefault();
      sendSerialParameter(input.dataset.serialParam);
    }
  });
});
async function applyTrajectory(persist){
  try{
    const settings={};
    document.querySelectorAll("[data-trajectory-key]").forEach(
      item=>settings[item.dataset.key]=Number(item.value)
    );
    await post({action:"settings",settings,persist});
    document.querySelectorAll("[data-trajectory-key]").forEach(
      item=>item.classList.remove("dirty")
    );
  }catch(error){notice(error.message,"bad");}
}
["shakeFrequency","shakeAmplitude"].forEach(id=>
  document.getElementById(id).addEventListener("input",updateShakeEstimate)
);
updateShakeEstimate();
async function applyAll(persist){
  try{
    const controlSettings={};
    controlFields.forEach(item=>controlSettings[item.dataset.key]=Number(item.value));
    const visionSettings={};
    visionFields.forEach(item=>visionSettings[item.dataset.visionKey]=Number(item.value));
    await Promise.all([
      post({action:"settings",settings:controlSettings,persist}),
      postVision({settings:visionSettings,persist})
    ]);
    [...controlFields,...visionFields].forEach(item=>item.classList.remove("dirty"));
    notice(persist?"全部参数已保存，重启后自动加载":"全部参数已应用");
    setTimeout(refresh,150);
  }catch(error){notice(error.message,"bad");}
}
async function refresh(){
  try{
    const response=await fetch("/status.json",{cache:"no-store"}); const d=await response.json();
    document.getElementById("mode").textContent=d.trajectory_active?"比赛轨迹":
      (d.control_enabled?"单环PD运行":"单环PD已停止");
    document.getElementById("axisState").textContent=!d.axis_referenced?"未回零":(d.driver_enabled?"保持使能":"急停释放");
    document.getElementById("ball").textContent=fmt(d.ball_position_cm,2," cm");
    document.getElementById("target").textContent=fmt(d.target_cm,2," cm");
    document.getElementById("posError").textContent=fmt(d.position_error_cm,2," cm");
    document.getElementById("velocity").textContent=fmt(d.velocity_cm_s,2," cm/s");
    document.getElementById("targetVelocity").textContent=fmt(d.target_velocity_cm_s,2," cm/s");
    document.getElementById("velError").textContent=fmt(d.velocity_error_cm_s,2," cm/s");
    document.getElementById("position").textContent=d.current_steps;
    document.getElementById("axisTarget").textContent=d.axis_target_steps;
    document.getElementById("controlCenter").textContent=d.control_center_steps;
    document.getElementById("controlMax").textContent=d.control_max_steps;
    const buttonLevel=value=>Number(value)===0?"按下(0)":"松开(1)";
    document.getElementById("buttonEmergency").textContent=buttonLevel(d.button_emergency_level);
    document.getElementById("buttonReference").textContent=buttonLevel(d.button_reference_level);
    document.getElementById("buttonZeroLoop").textContent=buttonLevel(d.button_zero_loop_level);
    document.getElementById("buttonTrajectory").textContent=buttonLevel(d.button_trajectory_level);
    document.getElementById("buttonCount").textContent=d.hardware_button_event_count??0;
    document.getElementById("buttonLastAction").textContent=
      d.hardware_button_last_error||d.hardware_button_last_action||"待机";
    const arbitraryNumber=document.getElementById("arbitraryTargetNumber");
    const arbitraryRange=document.getElementById("arbitraryTargetRange");
    if(document.activeElement!==arbitraryNumber&&document.activeElement!==arbitraryRange){
      arbitraryNumber.value=Number(d.target_cm).toFixed(1);
      arbitraryRange.value=d.target_cm;
      document.getElementById("arbitraryTargetReadout").textContent=Number(d.target_cm).toFixed(1)+" cm";
    }
    document.getElementById("driveSignalLevel").textContent=
      Number(d.drive_signal_level)?"高(1) 发车":"低(0) 停车";
    const driveEdges={idle:"待机",rising_start:"上升沿·向右发车",falling_stop:"下降沿·向左停车"};
    const drivePhases={idle:"待机",hold:"满角保持",fade:"正在回零"};
    document.getElementById("driveSignalEdge").textContent=
      driveEdges[d.drive_signal_edge]||d.drive_signal_edge||"待机";
    document.getElementById("driveFeedforwardPhase").textContent=
      drivePhases[d.drive_feedforward_phase]||d.drive_feedforward_phase||"待机";
    document.getElementById("driveFeedforwardAngle").textContent=
      fmt(d.drive_feedforward_angle_deg,2,"°");
    document.getElementById("driveFeedforwardSteps").textContent=
      fmt(d.drive_feedforward_steps,1,"步");
    document.getElementById("driveSignalCount").textContent=
      d.drive_feedforward_event_count??0;
    document.getElementById("serialState").textContent=
      d.serial_connected?"已连接":"未连接";
    document.getElementById("serialRxCount").textContent=
      `${d.serial_rx_bytes??0} B / ${d.serial_rx_lines_count??0}行`;
    document.getElementById("serialTxCount").textContent=
      `${d.serial_tx_bytes??0} B / ${d.serial_tx_messages??0}次`;
    document.getElementById("serialLastRx").textContent=d.serial_last_rx||"--";
    document.getElementById("serialError").textContent=d.serial_error||"";
    const serialParameters=d.serial_parameters&&typeof d.serial_parameters==="object"?d.serial_parameters:{};
    document.getElementById("serialParsedCount").textContent=Object.keys(serialParameters).length+"项";
    for(const name of SERIAL_PARAMETERS){
      const input=serialParameterInput(name);
      if(
        serialParameterHoldUntil[name]&&
        serialParameters[name]!==undefined&&
        Number(serialParameters[name])===Number(input.value)
      )delete serialParameterHoldUntil[name];
      const held=(serialParameterHoldUntil[name]||0)>Date.now();
      if(!held&&document.activeElement!==input&&serialParameters[name]!==undefined){
        input.value=serialParameters[name];
      }
    }
    const serialLines=Array.isArray(d.serial_rx_lines)?d.serial_rx_lines:[];
    const serialLog=document.getElementById("serialLog");
    const nextSerialText=serialLines.length?serialLines.join("\\n"):"等待串口数据…";
    if(serialLog.textContent!==nextSerialText){
      serialLog.textContent=nextSerialText;
      serialLog.scrollTop=serialLog.scrollHeight;
    }
    const escapeNames={idle:"待机",preload:"反向预加载",push:"单次破静摩擦",brake:"速度制动",cooldown:"串级接管冷却"};
    document.getElementById("escapeState").textContent=
      Number(d.escape_enabled)?(escapeNames[d.escape_phase]||d.escape_phase):"关闭";
    document.getElementById("integratorState").textContent=Number(d.control_mode)===1?
      (!Number(d.bias_enabled)?"关闭":(d.bias_learning?"正在学习":(d.bias_frozen?"冻结":"保持"))):
      (d.integrator_frozen?"积分冻结":"积分正常");
    document.getElementById("biasSteps").textContent=fmt(d.bias_steps,1," 步");
    document.getElementById("pdOutput").textContent=fmt(d.pd_output_steps,1," 步");
    document.getElementById("escapeMagnitude").textContent=fmt(Math.abs(d.escape_offset_steps||0),0," 步");
    document.getElementById("escapeCount").textContent=d.escape_count??0;
    document.getElementById("fps").textContent=fmt(d.vision_fps,1);
    const shakeNames={idle:"待机",fixed:"固定频率",sweep:"逐档扫频",returning:"正在回中心",stopping:"正在停止",error:"测试异常"};
    document.getElementById("shakeState").textContent=shakeNames[d.shake_phase]||d.shake_phase||"待机";
    document.getElementById("shakeCommand").textContent=d.shake_active?fmt(d.shake_command_frequency_hz,0," Hz"):"--";
    document.getElementById("shakeActualStep").textContent=fmt(d.shake_actual_step_rate_hz,0," Hz");
    document.getElementById("shakeActualCycle").textContent=fmt(d.shake_actual_cycle_hz,2," Hz");
    document.getElementById("shakeCycles").textContent=d.shake_cycles??0;
    document.getElementById("shakeStart").disabled=Boolean(d.shake_active);
    document.getElementById("shakeStop").disabled=!d.shake_active;
    const trajectoryNames={idle:"待机",centering:"确认0点并关PID",push_plus:"小角度开环向右",hold_minus:"-5闭环",complete:"完成",failed:"未通过",stopping:"正在停止",error:"异常"};
    document.getElementById("trajectoryPhase").textContent=trajectoryNames[d.trajectory_phase]||d.trajectory_phase||"待机";
    document.getElementById("trajectoryElapsed").textContent=fmt(d.trajectory_elapsed_s,2," s");
    document.getElementById("trajectoryOpenLoopSteps").textContent=
      d.trajectory_open_loop_origin_steps===null||d.trajectory_open_loop_origin_steps===undefined?"--":
      `${d.trajectory_open_loop_origin_steps}→${d.trajectory_open_loop_target_steps}步`;
    const takeoverNames={position:"+6提前保护",timeout:"固定时间到",official_timeout:"总时限保护"};
    document.getElementById("trajectoryTakeoverReason").textContent=
      takeoverNames[d.trajectory_takeover_reason]||"--";
    document.getElementById("trajectoryPlusPeak").textContent=fmt(d.trajectory_positive_peak_cm,2," cm");
    document.getElementById("trajectoryMinusPeak").textContent=fmt(d.trajectory_negative_peak_cm,2," cm");
    document.getElementById("trajectoryFinalError").textContent=fmt(d.trajectory_final_error_cm,2," cm");
    document.getElementById("trajectoryResult").textContent=d.trajectory_message||"尚未运行";
    document.getElementById("trajectoryStart").disabled=Boolean(d.trajectory_active||d.shake_active);
    document.getElementById("trajectoryStop").disabled=!d.trajectory_active;
    if(!controlInitialized){
      controlFields.forEach(item=>{if(d[item.dataset.key]!==undefined)item.value=d[item.dataset.key];});
      controlInitialized=true;
    }
    try{
      const visionResponse=await fetch(visionBase+"/status.json",{cache:"no-store"});
      const v=await visionResponse.json();
      document.getElementById("trackSource").textContent=
        v.track_source==="measured"?"真实检测":(v.track_source==="predicted"?"短时预测":"已丢失");
      const detectorNames={fused:"轮廓＋投影融合",contour:"二维轮廓",projection:"一维投影",none:"无候选"};
      document.getElementById("measurementSource").textContent=detectorNames[v.measurement_source]||"--";
      document.getElementById("trackConfidence").textContent=fmt(v.tracking_confidence,2);
      document.getElementById("projectionPeak").textContent=
        v.measurement_source==="projection"?fmt(v.projection_peak,0):"--";
      document.getElementById("missedFrames").textContent=v.missed_frames??"--";
      document.getElementById("pipeLeftX").textContent=fmt(v.pipe_left_x_px,1," px");
      document.getElementById("axisSync").textContent=
        v.axis_synchronized?"正常（"+v.axis_steps+"步）":"等待同步";
      document.getElementById("exposureView").textContent=v.exposure??"--";
      document.getElementById("cropView").textContent=Array.isArray(v.crop)?v.crop.join("～"):"--";
      if(!visionInitialized){
        const settingsResponse=await fetch(visionBase+"/settings.json",{cache:"no-store"});
        const settingsPayload=await settingsResponse.json();
        const s=settingsPayload.settings||{};
        visionFields.forEach(item=>{
          if(s[item.dataset.visionKey]!==undefined)item.value=s[item.dataset.visionKey];
        });
        visionInitialized=true;
      }
    }catch(visionError){
      document.getElementById("trackSource").textContent="视觉离线";
    }
    if(d.safety_reason) notice(d.safety_reason,"warn-text");
  }catch(error){notice("状态读取失败："+error.message,"bad");}
}
refresh(); setInterval(refresh,400);
</script>
</body>
</html>
"""


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class DirectStepperAxis:
    def __init__(
        self,
        minimum: int,
        balance: int,
        maximum: int,
        up_dir_level: int = 1,
    ) -> None:
        self.minimum = minimum
        self.balance = balance
        self.maximum = maximum
        self.up_dir_level = up_dir_level
        self.lock = threading.RLock()
        self.handle = lgpio.gpiochip_open(0)
        self.current_steps = 0
        self.target_steps = 0
        self.referenced = False
        self.driver_enabled = False
        self.step_frequency_hz = 300.0
        self.stop_event = threading.Event()
        try:
            lgpio.gpio_claim_output(self.handle, EN1, 0)
            lgpio.gpio_claim_output(self.handle, DIR1, 0)
            lgpio.gpio_claim_output(self.handle, STEP1, 0)
        except Exception:
            lgpio.gpiochip_close(self.handle)
            raise
        self.thread = threading.Thread(
            target=self._motion_loop,
            name="d36a-step-worker",
            daemon=True,
        )
        self.thread.start()

    def reference_zero(self) -> None:
        with self.lock:
            lgpio.gpio_write(self.handle, STEP1, 0)
            self.current_steps = self.minimum
            self.target_steps = self.minimum
            self.referenced = True
            lgpio.gpio_write(self.handle, EN1, 1)
            self.driver_enabled = True

    def resume(self) -> None:
        with self.lock:
            if not self.referenced:
                raise RuntimeError("轴尚未回零，请先把机构放到最低点并确认0")
            self.target_steps = self.current_steps
            lgpio.gpio_write(self.handle, STEP1, 0)
            lgpio.gpio_write(self.handle, EN1, 1)
            self.driver_enabled = True

    def emergency_stop(self) -> None:
        with self.lock:
            self.target_steps = self.current_steps
            lgpio.gpio_write(self.handle, STEP1, 0)
            lgpio.gpio_write(self.handle, EN1, 0)
            self.driver_enabled = False

    def set_target(self, target: int) -> None:
        with self.lock:
            if not self.referenced:
                raise RuntimeError("轴尚未回零")
            if not self.driver_enabled:
                raise RuntimeError("轴处于急停释放状态，请先恢复轴")
            self.target_steps = max(
                self.minimum,
                min(self.maximum, int(target)),
            )

    def set_frequency(self, frequency: float) -> None:
        with self.lock:
            self.step_frequency_hz = frequency

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "axis_referenced": self.referenced,
                "driver_enabled": self.driver_enabled,
                "current_steps": self.current_steps,
                "axis_target_steps": self.target_steps,
                "min_steps": self.minimum,
                "balance_steps": self.balance,
                "max_steps": self.maximum,
                "step_frequency_hz": self.step_frequency_hz,
            }

    def _motion_loop(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                ready = self.referenced and self.driver_enabled
                current = self.current_steps
                target = self.target_steps
                frequency = self.step_frequency_hz
            if not ready or current == target:
                self.stop_event.wait(0.002)
                continue

            direction_positive = target > current
            direction_level = (
                self.up_dir_level
                if direction_positive
                else 1 - self.up_dir_level
            )
            half_period = 0.5 / max(1.0, frequency)
            with self.lock:
                if not self.driver_enabled or not self.referenced:
                    continue
                lgpio.gpio_write(self.handle, DIR1, direction_level)
            time.sleep(0.00001)
            with self.lock:
                if not self.driver_enabled:
                    continue
                lgpio.gpio_write(self.handle, STEP1, 1)
                # The D36A advances on the rising edge, so update the
                # software position at the same instant.  An emergency stop
                # during the high half-cycle therefore preserves the count.
                self.current_steps += 1 if direction_positive else -1
                self.current_steps = max(
                    self.minimum,
                    min(self.maximum, self.current_steps),
                )
            self.stop_event.wait(half_period)
            with self.lock:
                lgpio.gpio_write(self.handle, STEP1, 0)
            self.stop_event.wait(half_period)

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)
        with self.lock:
            try:
                lgpio.gpio_write(self.handle, STEP1, 0)
                lgpio.gpio_write(self.handle, EN1, 0)
            finally:
                lgpio.gpiochip_close(self.handle)


class CascadeState:
    INTEGER_KEYS = {
        "control_mode",
        "bias_enabled",
        "stiction_ff_enabled",
        "max_offset_steps",
        "control_center_steps",
        "control_min_steps",
        "control_max_steps",
        "control_direction",
        "escape_enabled",
        "escape_direction",
        "escape_preload_steps",
        "escape_start_steps",
        "escape_push_steps",
        "escape_release_confirm_frames",
        "escape_brake_min_steps",
        "escape_recovery_steps",
        "trajectory_right_push_delta_steps",
        "trajectory_level_steps",
    }

    def __init__(self, settings_path: Path, axis: DirectStepperAxis) -> None:
        self.lock = threading.RLock()
        self.settings_path = settings_path
        self.axis = axis
        self.settings = dict(DEFAULT_SETTINGS)
        self.control_enabled = False
        self.manual_escape_requested = False
        self.safety_reason = "请人工放到最低位置，然后确认零点"
        self.reset_revision = 0
        self.runtime: dict[str, object] = {
            "ball_position_cm": None,
            "position_error_cm": None,
            "velocity_cm_s": None,
            "target_velocity_cm_s": 0.0,
            "velocity_error_cm_s": None,
            "position_integral": 0.0,
            "velocity_integral": 0.0,
            "pd_p_steps": 0.0,
            "pd_d_steps": 0.0,
            "pd_output_steps": 0.0,
            "pd_direction_gain": 1.0,
            "bias_steps": 0.0,
            "bias_learning": False,
            "bias_frozen": True,
            "control_output_steps": 0.0,
            "pid_output_steps": 0.0,
            "stiction_feedforward_steps": 0.0,
            "escape_active": False,
            "escape_phase": "idle",
            "escape_offset_steps": 0,
            "escape_count": 0,
            "integrator_frozen": False,
            "escape_release_frames": 0,
            "stationary_elapsed_s": 0.0,
            "vision_fps": None,
            "vision_age_s": None,
            "drive_signal_level": 0,
            "drive_signal_edge": "idle",
            "drive_feedforward_phase": "idle",
            "drive_feedforward_angle_deg": 0.0,
            "drive_feedforward_steps": 0.0,
            "drive_feedforward_event_count": 0,
        }
        self.drive_feedforward_started_at = 0.0
        self.drive_feedforward_sign = 0
        self._load()

    def _convert(self, payload: dict[str, object]) -> dict[str, int | float]:
        converted: dict[str, int | float] = {}
        for key, default in DEFAULT_SETTINGS.items():
            value = payload.get(key, default)
            converted[key] = (
                int(value) if key in self.INTEGER_KEYS else float(value)
            )
        converted["control_mode"] = 1
        return converted

    def _validate(self, values: dict[str, int | float]) -> None:
        checks = (
            ("target_cm", values["target_cm"], -12.5, 12.5),
            ("pd_kp_steps_per_cm", values["pd_kp_steps_per_cm"], 0.0, 1000.0),
            ("pd_left_gain", values["pd_left_gain"], 0.0, 5.0),
            ("pd_right_gain", values["pd_right_gain"], 0.0, 5.0),
            ("pd_kd_steps_per_cm_s", values["pd_kd_steps_per_cm_s"], 0.0, 1000.0),
            ("bias_ki_steps_per_cm_s", values["bias_ki_steps_per_cm_s"], 0.0, 100.0),
            ("bias_limit_steps", values["bias_limit_steps"], 0.0, 681.0),
            ("bias_zone_cm", values["bias_zone_cm"], 0.0, 12.5),
            ("bias_velocity_limit_cm_s", values["bias_velocity_limit_cm_s"], 0.0, 20.0),
            ("pos_kp", values["pos_kp"], 0.0, 10000.0),
            ("pos_ki", values["pos_ki"], 0.0, 10000.0),
            ("pos_kd", values["pos_kd"], 0.0, 10000.0),
            ("pos_integral_limit", values["pos_integral_limit"], 0.0, 100000.0),
            ("max_target_velocity_cm_s", values["max_target_velocity_cm_s"], 0.0, 1000.0),
            ("deadband_cm", values["deadband_cm"], 0.0, 12.5),
            ("pos_integral_zone_cm", values["pos_integral_zone_cm"], 0.0, 12.5),
            ("vel_kp", values["vel_kp"], 0.0, 10000.0),
            ("vel_ki", values["vel_ki"], 0.0, 10000.0),
            ("vel_kd", values["vel_kd"], 0.0, 10000.0),
            ("vel_integral_limit", values["vel_integral_limit"], 0.0, 100000.0),
            ("stiction_ff_positive_steps", values["stiction_ff_positive_steps"], 0.0, 681.0),
            ("stiction_ff_negative_steps", values["stiction_ff_negative_steps"], 0.0, 681.0),
            ("stiction_ff_fade_velocity_cm_s", values["stiction_ff_fade_velocity_cm_s"], 0.05, 50.0),
            ("max_offset_steps", values["max_offset_steps"], 1.0, 681.0),
            ("control_center_steps", values["control_center_steps"], 0.0, 681.0),
            ("control_min_steps", values["control_min_steps"], 0.0, 680.0),
            ("control_max_steps", values["control_max_steps"], 1.0, 681.0),
            ("velocity_alpha", values["velocity_alpha"], 0.001, 1.0),
            ("control_hz", values["control_hz"], 1.0, 100.0),
            ("step_frequency_hz", values["step_frequency_hz"], 1.0, 5000.0),
            ("vision_timeout_s", values["vision_timeout_s"], 0.01, 30.0),
            ("drive_ff_angle_deg", values["drive_ff_angle_deg"], 0.0, 90.0),
            ("drive_ff_steps_per_degree", values["drive_ff_steps_per_degree"], 0.0, 681.0),
            ("drive_ff_hold_s", values["drive_ff_hold_s"], 0.0, 30.0),
            ("drive_ff_fade_s", values["drive_ff_fade_s"], 0.01, 30.0),
            ("trajectory_level_steps", values["trajectory_level_steps"], 0.0, 681.0),
            ("trajectory_right_push_delta_steps", values["trajectory_right_push_delta_steps"], 1.0, 200.0),
            ("trajectory_takeover_cm", values["trajectory_takeover_cm"], 4.0, 6.0),
            ("trajectory_open_loop_timeout_s", values["trajectory_open_loop_timeout_s"], 0.2, 4.0),
            ("trajectory_open_loop_frequency_hz", values["trajectory_open_loop_frequency_hz"], 20.0, 5000.0),
            ("escape_velocity_threshold_cm_s", values["escape_velocity_threshold_cm_s"], 0.0, 100.0),
            ("escape_error_threshold_cm", values["escape_error_threshold_cm"], 0.0, 12.5),
            ("escape_stationary_s", values["escape_stationary_s"], 0.01, 60.0),
            ("escape_preload_steps", values["escape_preload_steps"], 0.0, 681.0),
            ("escape_preload_s", values["escape_preload_s"], 0.0, 5.0),
            ("escape_start_steps", values["escape_start_steps"], 1.0, 681.0),
            ("escape_ramp_steps_per_s", values["escape_ramp_steps_per_s"], 0.0, 5000.0),
            ("escape_push_steps", values["escape_push_steps"], 1.0, 681.0),
            ("escape_release_velocity_cm_s", values["escape_release_velocity_cm_s"], 0.0, 100.0),
            ("escape_release_displacement_cm", values["escape_release_displacement_cm"], 0.0, 12.5),
            ("escape_release_confirm_frames", values["escape_release_confirm_frames"], 1.0, 30.0),
            ("escape_push_max_s", values["escape_push_max_s"], 0.01, 30.0),
            ("escape_brake_min_steps", values["escape_brake_min_steps"], 0.0, 681.0),
            ("escape_brake_gain_steps_per_cm_s", values["escape_brake_gain_steps_per_cm_s"], 0.0, 1000.0),
            ("escape_recovery_steps", values["escape_recovery_steps"], 1.0, 681.0),
            ("escape_recovery_velocity_cm_s", values["escape_recovery_velocity_cm_s"], 0.0, 100.0),
            ("escape_recovery_min_s", values["escape_recovery_min_s"], 0.0, 30.0),
            ("escape_recovery_max_s", values["escape_recovery_max_s"], 0.01, 30.0),
            ("escape_cooldown_s", values["escape_cooldown_s"], 0.0, 120.0),
        )
        for name, value, lower, upper in checks:
            numeric = float(value)
            if not math.isfinite(numeric) or not lower <= numeric <= upper:
                raise ValueError(f"{name}必须在{lower:g}～{upper:g}范围内")
        if int(values["control_direction"]) not in (-1, 1):
            raise ValueError("control_direction只能为-1或+1")
        if int(values["control_mode"]) != 1:
            raise ValueError("本版本固定使用单环PD")
        if int(values["bias_enabled"]) not in (0, 1):
            raise ValueError("bias_enabled只能为0或1")
        if int(values["stiction_ff_enabled"]) not in (0, 1):
            raise ValueError("stiction_ff_enabled只能为0或1")
        if int(values["escape_enabled"]) not in (0, 1):
            raise ValueError("escape_enabled只能为0或1")
        if int(values["escape_direction"]) not in (-1, 1):
            raise ValueError("escape_direction只能为-1或+1")
        control_min = int(values["control_min_steps"])
        control_center = int(values["control_center_steps"])
        control_max = int(values["control_max_steps"])
        if not (
            self.axis.minimum
            <= control_min
            <= control_center
            <= control_max
            <= self.axis.maximum
        ):
            raise ValueError("控制步数必须满足0≤最小≤中心≤最大≤681")
        if float(values["escape_recovery_min_s"]) > float(
            values["escape_recovery_max_s"]
        ):
            raise ValueError("最短制动时间不能大于最长制动时间")
        if int(values["escape_start_steps"]) > int(values["escape_push_steps"]):
            raise ValueError("破静摩擦起始幅度不能大于最大幅度")
        if int(values["escape_brake_min_steps"]) > int(
            values["escape_recovery_steps"]
        ):
            raise ValueError("最小制动幅度不能大于最大制动幅度")
        step_frequency = float(values["step_frequency_hz"])
        push_move_s = int(values["escape_push_steps"]) / step_frequency
        if push_move_s > float(values["escape_push_max_s"]) * 0.9:
            raise ValueError("破静摩擦最大幅度在当前步进频率和最大推离时间内无法到达")
        recovery_move_s = (
            int(values["escape_push_steps"])
            + int(values["escape_recovery_steps"])
        ) / step_frequency
        if recovery_move_s > float(values["escape_recovery_max_s"]) * 0.9:
            raise ValueError("最大制动幅度在当前步进频率和最长制动时间内无法到达")

    def _load(self) -> None:
        if not self.settings_path.exists():
            return
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
            values = self._convert(payload)
            self._validate(values)
            self.settings.update(values)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            print(f"Ignoring invalid settings: {error}", flush=True)
        self.axis.set_frequency(float(self.settings["step_frequency_hz"]))

    def update_settings(
        self,
        payload: dict[str, object],
        persist: bool,
    ) -> None:
        with self.lock:
            merged = dict(self.settings)
            for key in DEFAULT_SETTINGS:
                if key in payload:
                    merged[key] = payload[key]
            converted = self._convert(merged)
            self._validate(converted)
            self.settings.update(converted)
            self.axis.set_frequency(float(self.settings["step_frequency_hz"]))
            self.reset_revision += 1
            self.safety_reason = ""
            if persist:
                self.settings_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.settings_path.with_suffix(
                    self.settings_path.suffix + ".tmp"
                )
                temporary.write_text(
                    json.dumps(self.settings, ensure_ascii=False, indent=2)
                    + "\n",
                    encoding="utf-8",
                )
                temporary.replace(self.settings_path)

    def set_target(self, target: float) -> None:
        self.update_settings({"target_cm": target}, False)

    def request_escape(self) -> None:
        with self.lock:
            axis = self.axis.snapshot()
            if not self.control_enabled:
                raise RuntimeError("请先开启闭环，再测试大角度脱困")
            if not axis["axis_referenced"] or not axis["driver_enabled"]:
                raise RuntimeError("轴未回零或处于急停释放状态")
            self.manual_escape_requested = True

    def consume_escape_request(self) -> bool:
        with self.lock:
            requested = self.manual_escape_requested
            self.manual_escape_requested = False
            return requested

    def trigger_drive_signal(self, level: int) -> None:
        """Start a right launch pulse or a left stop pulse on a clean edge."""
        with self.lock:
            level = 1 if int(level) else 0
            self.drive_feedforward_sign = 1 if level else -1
            self.drive_feedforward_started_at = time.monotonic()
            angle = (
                self.drive_feedforward_sign
                * float(self.settings["drive_ff_angle_deg"])
            )
            steps = (
                int(self.settings["control_direction"])
                * angle
                * float(self.settings["drive_ff_steps_per_degree"])
            )
            self.runtime["drive_signal_level"] = level
            self.runtime["drive_signal_edge"] = (
                "rising_start" if level else "falling_stop"
            )
            self.runtime["drive_feedforward_phase"] = "hold"
            self.runtime["drive_feedforward_angle_deg"] = angle
            self.runtime["drive_feedforward_steps"] = steps
            self.runtime["drive_feedforward_event_count"] = (
                int(self.runtime["drive_feedforward_event_count"]) + 1
            )

    def drive_feedforward(self, now: float) -> float:
        with self.lock:
            if self.drive_feedforward_sign == 0:
                return 0.0
            elapsed = max(0.0, now - self.drive_feedforward_started_at)
            hold_s = float(self.settings["drive_ff_hold_s"])
            fade_s = float(self.settings["drive_ff_fade_s"])
            if elapsed <= hold_s:
                scale = 1.0
                phase = "hold"
            elif elapsed <= hold_s + fade_s:
                scale = 1.0 - (elapsed - hold_s) / fade_s
                phase = "fade"
            else:
                scale = 0.0
                phase = "idle"
                self.drive_feedforward_sign = 0
            angle = (
                self.drive_feedforward_sign
                * float(self.settings["drive_ff_angle_deg"])
                * scale
            )
            steps = (
                int(self.settings["control_direction"])
                * angle
                * float(self.settings["drive_ff_steps_per_degree"])
            )
            self.runtime["drive_feedforward_phase"] = phase
            self.runtime["drive_feedforward_angle_deg"] = angle
            self.runtime["drive_feedforward_steps"] = steps
            return steps

    def control_center(self) -> int:
        with self.lock:
            return int(self.settings["control_center_steps"])

    def enable(self) -> None:
        with self.lock:
            axis = self.axis.snapshot()
            if not axis["axis_referenced"]:
                raise RuntimeError("请先把机构放到最低位置并确认零点")
            if not axis["driver_enabled"]:
                raise RuntimeError("轴处于急停释放状态，请先恢复轴")
            if self.runtime["ball_position_cm"] is None:
                raise RuntimeError("当前没有识别到小球")
            self.control_enabled = True
            self.reset_revision += 1
            self.safety_reason = ""

    def disable(self, reason: str = "") -> None:
        with self.lock:
            self.control_enabled = False
            self.manual_escape_requested = False
            self.reset_revision += 1
            self.safety_reason = reason
            axis = self.axis.snapshot()
            if axis["axis_referenced"] and axis["driver_enabled"]:
                self.axis.set_target(int(axis["current_steps"]))

    def set_control_target(self, desired: int, expected_revision: int) -> bool:
        """Send a controller command only while its state snapshot is current."""
        with self.lock:
            if (
                not self.control_enabled
                or self.reset_revision != expected_revision
            ):
                return False
            self.axis.set_target(desired)
            return True

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                **self.settings,
                **self.runtime,
                **self.axis.snapshot(),
                "control_enabled": self.control_enabled,
                "safety_reason": self.safety_reason,
                "control_scheme": "single-pd-bias-escape-direct-stepper",
            }

    def set_runtime(self, values: dict[str, object]) -> None:
        with self.lock:
            self.runtime.update(values)

    def settings_snapshot(self) -> tuple[dict[str, int | float], bool, int]:
        with self.lock:
            return dict(self.settings), self.control_enabled, self.reset_revision


class StepperShakeTest:
    """Run a bounded triangle-wave position test without fighting the PID."""

    SWEEP_FREQUENCIES = (
        20,
        50,
        100,
        200,
        300,
        450,
        600,
        800,
        1000,
        1200,
        1500,
        2000,
        2500,
        3000,
        4000,
        5000,
    )

    def __init__(self, state: CascadeState, axis: DirectStepperAxis) -> None:
        self.state = state
        self.axis = axis
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.abort_return_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.active = False
        self.phase = "idle"
        self.mode = "fixed"
        self.center_steps = 0
        self.amplitude_steps = 0
        self.command_frequency_hz: float | None = None
        self.actual_step_rate_hz: float | None = None
        self.actual_cycle_hz: float | None = None
        self.cycles = 0
        self.stage_index = 0
        self.stage_count = 0
        self.error = ""
        self.saved_frequency_hz = 300.0

    def is_active(self) -> bool:
        with self.lock:
            return self.active

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "shake_active": self.active,
                "shake_phase": self.phase,
                "shake_mode": self.mode,
                "shake_center_steps": self.center_steps,
                "shake_amplitude_steps": self.amplitude_steps,
                "shake_command_frequency_hz": self.command_frequency_hz,
                "shake_actual_step_rate_hz": self.actual_step_rate_hz,
                "shake_actual_cycle_hz": self.actual_cycle_hz,
                "shake_cycles": self.cycles,
                "shake_stage_index": self.stage_index,
                "shake_stage_count": self.stage_count,
                "shake_error": self.error,
            }

    @staticmethod
    def _number(
        payload: dict[str, object],
        key: str,
        minimum: float,
        maximum: float,
    ) -> float:
        value = float(payload.get(key))
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{key}必须在{minimum:g}～{maximum:g}范围内")
        return value

    def start(self, payload: dict[str, object]) -> dict[str, object]:
        mode = str(payload.get("mode", "fixed"))
        if mode not in ("fixed", "sweep"):
            raise ValueError("测试模式只能为fixed或sweep")
        frequency = self._number(payload, "frequency_hz", 20.0, 5000.0)
        amplitude = int(
            self._number(payload, "amplitude_steps", 1.0, 300.0)
        )
        duration = self._number(payload, "duration_s", 0.5, 30.0)
        sweep_start = self._number(
            payload, "sweep_start_hz", 20.0, 5000.0
        )
        sweep_end = self._number(
            payload, "sweep_end_hz", 20.0, 5000.0
        )
        sweep_dwell = self._number(
            payload, "sweep_dwell_s", 0.3, 10.0
        )
        if sweep_start > sweep_end:
            raise ValueError("扫频起点不能高于终点")

        axis = self.axis.snapshot()
        if not axis["axis_referenced"]:
            raise RuntimeError("轴尚未回零，不能开始抖动测试")
        if not axis["driver_enabled"]:
            raise RuntimeError("轴处于急停释放状态，请先恢复轴")
        center = int(axis["current_steps"])
        safe_amplitude = min(
            center - int(axis["min_steps"]),
            int(axis["max_steps"]) - center,
        )
        if amplitude > safe_amplitude:
            raise ValueError(
                f"当前位置{center}步最多允许±{safe_amplitude}步，"
                "请减小幅度或先移动到更居中的位置"
            )

        if mode == "fixed":
            stages = [(frequency, duration)]
        else:
            frequencies = sorted(
                {
                    sweep_start,
                    sweep_end,
                    *(
                        float(item)
                        for item in self.SWEEP_FREQUENCIES
                        if sweep_start <= item <= sweep_end
                    ),
                }
            )
            stages = [(item, sweep_dwell) for item in frequencies]
            if sum(item[1] for item in stages) > 120.0:
                raise ValueError("扫频总时间不能超过120秒")

        with self.lock:
            if self.active:
                raise RuntimeError("抖动测试已经在运行")
            self.state.disable("步进抖动测试运行中，闭环已暂停")
            self.stop_event.clear()
            self.abort_return_event.clear()
            self.active = True
            self.phase = mode
            self.mode = mode
            self.center_steps = center
            self.amplitude_steps = amplitude
            self.command_frequency_hz = stages[0][0]
            self.actual_step_rate_hz = None
            self.actual_cycle_hz = None
            self.cycles = 0
            self.stage_index = 0
            self.stage_count = len(stages)
            self.error = ""
            self.saved_frequency_hz = float(axis["step_frequency_hz"])
            self.thread = threading.Thread(
                target=self._run,
                args=(stages,),
                name="d36a-shake-test",
                daemon=True,
            )
            self.thread.start()
        return {
            "mode": mode,
            "center_steps": center,
            "amplitude_steps": amplitude,
            "stage_count": len(stages),
        }

    def stop(self) -> bool:
        with self.lock:
            if not self.active:
                return False
            self.phase = "stopping"
            self.stop_event.set()
            return True

    def cancel_immediately(self) -> None:
        with self.lock:
            self.abort_return_event.set()
            self.stop_event.set()

    def close(self) -> None:
        self.cancel_immediately()
        thread = self.thread
        if thread is not None:
            thread.join(timeout=2)

    def _set_frequency(self, frequency: float, stage_index: int) -> None:
        self.axis.set_frequency(frequency)
        with self.lock:
            self.command_frequency_hz = frequency
            self.stage_index = stage_index

    def _run_stage(self, frequency: float, duration: float) -> None:
        center = self.center_steps
        amplitude = self.amplitude_steps
        targets = (center + amplitude, center - amplitude)
        deadline = time.monotonic() + duration
        target_index = 0
        reached_endpoints = 0
        full_leg_times: list[float] = []

        while time.monotonic() < deadline and not self.stop_event.is_set():
            target = targets[target_index]
            start_axis = self.axis.snapshot()
            start_position = int(start_axis["current_steps"])
            started_at = time.monotonic()
            self.axis.set_target(target)
            reached = False
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.001):
                    break
                axis = self.axis.snapshot()
                if not axis["driver_enabled"]:
                    raise RuntimeError("测试期间驱动被释放")
                if int(axis["current_steps"]) == target:
                    reached = True
                    break
            if not reached:
                break

            elapsed = max(0.000001, time.monotonic() - started_at)
            moved_steps = abs(target - start_position)
            if moved_steps:
                with self.lock:
                    self.actual_step_rate_hz = moved_steps / elapsed

            reached_endpoints += 1
            # The first center-to-edge leg is only half an oscillation.  Every
            # two full edge-to-edge legs after it form one complete cycle.
            if reached_endpoints > 1:
                full_leg_times.append(elapsed)
                if len(full_leg_times) == 2:
                    cycle_time = sum(full_leg_times)
                    with self.lock:
                        self.actual_cycle_hz = 1.0 / max(
                            0.000001, cycle_time
                        )
                        self.cycles += 1
                    full_leg_times.clear()
            target_index = 1 - target_index

    def _return_to_center(self) -> None:
        with self.lock:
            self.phase = "returning"
        axis = self.axis.snapshot()
        if not axis["axis_referenced"] or not axis["driver_enabled"]:
            return
        self.axis.set_target(self.center_steps)
        return_deadline = time.monotonic() + 5.0
        while time.monotonic() < return_deadline:
            axis = self.axis.snapshot()
            if int(axis["current_steps"]) == self.center_steps:
                return
            if not axis["driver_enabled"] or self.abort_return_event.is_set():
                return
            time.sleep(0.002)

    def _run(self, stages: list[tuple[float, float]]) -> None:
        error = ""
        try:
            for index, (frequency, duration) in enumerate(stages, start=1):
                if self.stop_event.is_set():
                    break
                self._set_frequency(frequency, index)
                with self.lock:
                    self.phase = self.mode
                self._run_stage(frequency, duration)
        except (RuntimeError, OSError) as caught:
            error = str(caught)
        finally:
            if not self.abort_return_event.is_set():
                try:
                    self._return_to_center()
                except (RuntimeError, OSError) as caught:
                    error = error or str(caught)
            try:
                self.axis.set_frequency(self.saved_frequency_hz)
            except (RuntimeError, OSError):
                pass
            with self.lock:
                self.active = False
                self.error = error
                self.phase = "error" if error else "idle"
                self.command_frequency_hz = None
            if not self.abort_return_event.is_set():
                reason = (
                    f"步进抖动测试异常：{error}"
                    if error
                    else "步进抖动测试结束，已回测试中心；闭环保持关闭"
                )
                self.state.disable(reason)


class ScoredTrajectoryTest:
    """One-click O -> +5 cm -> -5 cm scored motion.

    The operator first stabilizes the ball at O with normal control.  This
    routine then disables PID, moves to the separately calibrated trajectory
    level, adds a small relative rightward step offset, and holds it for the
    configured fixed open-loop duration.  A positive-position guard may hand
    over early only to prevent the ball from exceeding +6 cm.
    """

    TUNED_SETTINGS: dict[str, int | float] = {
        "control_mode": 1,
        "pd_kp_steps_per_cm": 14.0,
        "pd_kd_steps_per_cm_s": 25.0,
        "bias_enabled": 1,
        "bias_ki_steps_per_cm_s": 40.0,
        "bias_limit_steps": 35.0,
        "bias_zone_cm": 6.0,
        "bias_velocity_limit_cm_s": 0.5,
        "max_offset_steps": 220,
        "velocity_alpha": 0.35,
        "control_hz": 25.0,
        "step_frequency_hz": 1500.0,
        "escape_enabled": 0,
    }
    START_TOLERANCE_CM = 1.0
    OFFICIAL_DURATION_S = 5.0
    TARGET_NEGATIVE_CM = -5.0

    def __init__(self, state: CascadeState, axis: DirectStepperAxis) -> None:
        self.state = state
        self.axis = axis
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.active = False
        self.phase = "idle"
        self.official_elapsed_s = 0.0
        self.positive_reach_s: float | None = None
        self.positive_peak_cm: float | None = None
        self.negative_peak_cm: float | None = None
        self.final_position_cm: float | None = None
        self.final_error_cm: float | None = None
        self.tail_max_error_cm: float | None = None
        self.open_loop_origin_steps: int | None = None
        self.open_loop_target_steps: int | None = None
        self.takeover_reason = ""
        self.passed: bool | None = None
        self.message = "待机"

    def is_active(self) -> bool:
        with self.lock:
            return self.active

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "trajectory_active": self.active,
                "trajectory_phase": self.phase,
                "trajectory_elapsed_s": self.official_elapsed_s,
                "trajectory_positive_reach_s": self.positive_reach_s,
                "trajectory_positive_peak_cm": self.positive_peak_cm,
                "trajectory_negative_peak_cm": self.negative_peak_cm,
                "trajectory_final_position_cm": self.final_position_cm,
                "trajectory_final_error_cm": self.final_error_cm,
                "trajectory_tail_max_error_cm": self.tail_max_error_cm,
                "trajectory_open_loop_origin_steps": self.open_loop_origin_steps,
                "trajectory_open_loop_target_steps": self.open_loop_target_steps,
                "trajectory_takeover_reason": self.takeover_reason,
                "trajectory_passed": self.passed,
                "trajectory_message": self.message,
            }

    def start(self) -> None:
        axis = self.axis.snapshot()
        if not axis["axis_referenced"]:
            raise RuntimeError("轴尚未回零，不能开始一键轨迹")
        if not axis["driver_enabled"]:
            raise RuntimeError("轴处于急停释放状态，请先恢复轴")
        state = self.state.snapshot()
        if state.get("ball_position_cm") is None:
            raise RuntimeError("当前没有识别到小球，请先把小球放在 O 点附近")
        if abs(float(state["ball_position_cm"])) > self.START_TOLERANCE_CM:
            raise RuntimeError("小球必须放在 O 点±1 cm内才能启动本题")

        with self.lock:
            if self.active:
                raise RuntimeError("一键轨迹已经在运行")
            self.stop_event.clear()
            self.active = True
            self.phase = "push_plus"
            self.official_elapsed_s = 0.0
            self.positive_reach_s = None
            self.positive_peak_cm = None
            self.negative_peak_cm = None
            self.final_position_cm = None
            self.final_error_cm = None
            self.tail_max_error_cm = None
            self.open_loop_origin_steps = None
            self.open_loop_target_steps = None
            self.takeover_reason = ""
            self.passed = None
            self.message = "控制键已接收：正在关闭普通闭环并立即启动开环"
            self.thread = threading.Thread(
                target=self._run,
                name="d36a-scored-trajectory",
                daemon=True,
            )
            self.thread.start()

    def stop(self) -> bool:
        with self.lock:
            if not self.active:
                return False
            self.phase = "stopping"
            self.message = "正在停止一键轨迹"
            self.stop_event.set()
            return True

    def cancel_immediately(self) -> None:
        self.stop_event.set()

    def close(self) -> None:
        self.cancel_immediately()
        thread = self.thread
        if thread is not None:
            thread.join(timeout=2)

    def _sample(self) -> tuple[float | None, float | None]:
        state = self.state.snapshot()
        position = state.get("ball_position_cm")
        velocity = state.get("velocity_cm_s")
        return (
            float(position) if position is not None else None,
            float(velocity) if velocity is not None else None,
        )

    def _run_official(self) -> None:
        settings, _enabled, _revision = self.state.settings_snapshot()
        origin_steps = int(settings["trajectory_level_steps"])
        right_push_delta_steps = int(
            settings["trajectory_right_push_delta_steps"]
        )
        takeover_cm = float(settings["trajectory_takeover_cm"])
        open_loop_timeout_s = float(
            settings["trajectory_open_loop_timeout_s"]
        )
        open_loop_frequency_hz = float(
            settings["trajectory_open_loop_frequency_hz"]
        )
        # This scored task owns the actuator as soon as the button is pressed.
        # disable() invalidates any in-flight ordinary-controller snapshot; the
        # controller's revision-checked command path cannot overwrite the open
        # loop target after this point.
        self.state.disable("")
        axis = self.axis.snapshot()
        # The calibrated trajectory level is intentionally independent from
        # the ordinary controller's nominal 235-step center and learned bias.
        # Using it for the -5 controller reproduces the tested 314 -> 374 move.
        trajectory_settings = dict(self.TUNED_SETTINGS)
        trajectory_settings["control_center_steps"] = origin_steps
        self.state.update_settings(trajectory_settings, False)
        axis_direction = int(settings["control_direction"])
        open_loop_target_steps = int(
            round(
                clamp(
                    origin_steps + axis_direction * right_push_delta_steps,
                    int(axis["min_steps"]),
                    int(axis["max_steps"]),
                )
            )
        )
        self.axis.set_frequency(open_loop_frequency_hz)
        started_at = time.monotonic()
        self.axis.set_target(open_loop_target_steps)
        with self.lock:
            self.open_loop_origin_steps = origin_steps
            self.open_loop_target_steps = open_loop_target_steps
            self.phase = "push_plus"
            self.message = (
                f"PID已关闭：从{origin_steps}步向右增加"
                f"{right_push_delta_steps}步；以{origin_steps}步为本次闭环中心，"
                f"固定开环{open_loop_timeout_s:g}秒，{takeover_cm:+g} cm提前保护"
            )

        lost_since: float | None = None
        tail_max_error = 0.0
        positive_peak = -math.inf
        negative_peak = math.inf

        while True:
            if self.stop_event.wait(0.008):
                raise RuntimeError("用户停止")
            now = time.monotonic()
            elapsed = now - started_at
            position, velocity = self._sample()
            with self.lock:
                self.official_elapsed_s = min(
                    elapsed, self.OFFICIAL_DURATION_S
                )

            if position is None or velocity is None:
                lost_since = lost_since or now
                if now - lost_since > 0.25:
                    raise RuntimeError("正式轨迹视觉持续丢失")
                if elapsed >= self.OFFICIAL_DURATION_S:
                    raise RuntimeError("5秒结束时没有小球坐标")
                continue
            lost_since = None
            positive_peak = max(positive_peak, position)
            negative_peak = min(negative_peak, position)
            if elapsed >= 4.5:
                tail_max_error = max(
                    tail_max_error,
                    abs(position - self.TARGET_NEGATIVE_CM),
                )

            with self.lock:
                phase = self.phase
            takeover_by_position = position >= takeover_cm
            takeover_by_timeout = elapsed >= open_loop_timeout_s
            if phase == "push_plus" and (
                takeover_by_position or takeover_by_timeout
            ):
                with self.lock:
                    self.positive_reach_s = elapsed
                    self.takeover_reason = (
                        "position" if takeover_by_position else "timeout"
                    )
                self.axis.set_frequency(
                    float(self.TUNED_SETTINGS["step_frequency_hz"])
                )
                self.state.set_target(self.TARGET_NEGATIVE_CM)
                self.state.enable()
                with self.lock:
                    self.phase = "hold_minus"
                    self.message = (
                        f"已在{position:+.2f} cm结束开环"
                        f"（{'正向超程保护' if takeover_by_position else '固定时间到'}），"
                        "单环PD闭环稳定到-5 cm"
                    )

            if elapsed >= self.OFFICIAL_DURATION_S:
                break

        if self.phase != "hold_minus":
            # Never leave the actuator in open loop if the configured switch
            # point was not reached inside the five-second window.
            self.axis.set_frequency(
                float(self.TUNED_SETTINGS["step_frequency_hz"])
            )
            self.state.set_target(self.TARGET_NEGATIVE_CM)
            self.state.enable()
            with self.lock:
                self.phase = "hold_minus"
                self.takeover_reason = "official_timeout"
                self.message = "5秒内未到切换点，已强制转入-5 cm闭环"

        position, _velocity = self._sample()
        if position is None:
            raise RuntimeError("5秒结束时没有小球坐标")
        final_error = abs(position - self.TARGET_NEGATIVE_CM)
        positive_error = abs(positive_peak - 5.0)
        negative_error = abs(negative_peak + 5.0)
        passed = (
            self.phase == "hold_minus"
            and positive_error <= 1.0
            and negative_error <= 1.0
            and final_error <= 1.0
            and tail_max_error <= 1.0
        )
        with self.lock:
            self.positive_peak_cm = positive_peak
            self.negative_peak_cm = negative_peak
            self.final_position_cm = position
            self.final_error_cm = final_error
            self.tail_max_error_cm = tail_max_error
            self.passed = passed
            self.phase = "complete" if passed else "failed"
            self.message = (
                "通过：5秒内完成并稳定在 -5±1 cm"
                if passed
                else "未通过：请查看峰值、末值和末段最大误差"
            )

    def _run(self) -> None:
        error = ""
        stopped = False
        try:
            self._run_official()
        except (RuntimeError, OSError, ValueError) as caught:
            error = str(caught)
            stopped = self.stop_event.is_set()
        finally:
            try:
                self.axis.set_frequency(
                    float(self.TUNED_SETTINGS["step_frequency_hz"])
                )
            except (RuntimeError, OSError):
                pass
            if error:
                self.state.disable(error)
            with self.lock:
                self.active = False
                if error:
                    self.passed = False
                    self.phase = "idle" if stopped else "error"
                    self.message = error


class CascadeController:
    def __init__(
        self,
        state: CascadeState,
        axis: DirectStepperAxis,
        vision_status_url: str,
    ) -> None:
        self.state = state
        self.axis = axis
        self.vision_status_url = vision_status_url
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="cascade-controller",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)

    def _vision(self) -> tuple[float | None, float | None, float | None]:
        try:
            with urllib.request.urlopen(
                self.vision_status_url,
                timeout=0.35,
            ) as response:
                payload = json.load(response)
            position = (
                float(payload["position_cm"])
                if payload.get("position_cm") is not None
                else None
            )
            fps = (
                float(payload["fps"])
                if payload.get("fps") is not None
                else None
            )
            timestamp = (
                float(payload["timestamp"])
                if payload.get("timestamp") is not None
                else None
            )
            return position, fps, timestamp
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None, None, None

    def _run(self) -> None:
        last_control = time.monotonic()
        last_ball: float | None = None
        last_seen = last_control
        filtered_velocity = 0.0
        position_integral = 0.0
        velocity_integral = 0.0
        bias_steps = 0.0
        previous_velocity_error: float | None = None
        observed_revision = -1
        stationary_since: float | None = None
        escape_phase = "idle"
        escape_phase_started_at = 0.0
        escape_ball_direction = 1
        escape_axis_direction = 1
        escape_start_ball: float | None = None
        escape_release_frames = 0
        escape_brake_ball_direction = 1
        last_escape_offset_steps = 0
        bumpless_output_steps: float | None = None
        escape_cooldown_until = 0.0
        escape_count = 0

        while not self.stop_event.is_set():
            cycle_start = time.monotonic()
            dt = max(0.001, cycle_start - last_control)
            last_control = cycle_start
            settings, enabled, revision = self.state.settings_snapshot()

            if revision != observed_revision:
                # A newly enabled target must not inherit stale velocity.
                # Start from zero and let the next real frame re-establish it.
                last_ball = None
                filtered_velocity = 0.0
                last_seen = cycle_start
                position_integral = 0.0
                velocity_integral = 0.0
                bias_steps = 0.0
                previous_velocity_error = None
                stationary_since = None
                escape_phase = "idle"
                escape_phase_started_at = 0.0
                escape_start_ball = None
                escape_release_frames = 0
                last_escape_offset_steps = 0
                bumpless_output_steps = None
                escape_cooldown_until = 0.0
                observed_revision = revision
            manual_escape_requested = self.state.consume_escape_request()
            drive_feedforward_steps = self.state.drive_feedforward(
                cycle_start
            )
            drive_feedforward_active = (
                abs(drive_feedforward_steps) >= 0.01
            )

            ball, vision_fps, vision_timestamp = self._vision()
            if ball is not None:
                raw_velocity = (
                    0.0
                    if last_ball is None
                    else clamp((ball - last_ball) / dt, -80.0, 80.0)
                )
                alpha = float(settings["velocity_alpha"])
                filtered_velocity = (
                    (1.0 - alpha) * filtered_velocity
                    + alpha * raw_velocity
                )
                last_ball = ball
                last_seen = cycle_start
            else:
                filtered_velocity *= 0.85

            position_error = (
                float(settings["target_cm"]) - ball
                if ball is not None
                else None
            )
            target_velocity = 0.0
            velocity_error: float | None = None
            output_steps = 0.0
            pid_output_steps = 0.0
            stiction_feedforward_steps = 0.0
            pd_p_steps = 0.0
            pd_d_steps = 0.0
            pd_output_steps = 0.0
            pd_direction_gain = 1.0
            bias_learning = False
            bias_frozen = True
            escape_active = False
            escape_offset_steps = 0
            integrator_frozen = False
            stationary_elapsed_s = 0.0

            if enabled:
                if cycle_start - last_seen > float(settings["vision_timeout_s"]):
                    control_center = int(settings["control_center_steps"])
                    self.state.disable(
                        f"视觉丢失，闭环已停止并回控制中心{control_center}"
                    )
                    try:
                        self.axis.set_target(control_center)
                    except RuntimeError:
                        pass
                    enabled = False
                elif ball is not None and position_error is not None:
                    stationary_condition = (
                        abs(filtered_velocity)
                        <= float(settings["escape_velocity_threshold_cm_s"])
                        and abs(position_error)
                        >= float(settings["escape_error_threshold_cm"])
                    )

                    if (
                        escape_phase == "cooldown"
                        and cycle_start >= escape_cooldown_until
                    ):
                        escape_phase = "idle"
                        escape_phase_started_at = cycle_start

                    if escape_phase == "preload":
                        phase_elapsed = cycle_start - escape_phase_started_at
                        if phase_elapsed >= float(settings["escape_preload_s"]):
                            escape_phase = "push"
                            escape_phase_started_at = cycle_start
                            escape_start_ball = ball
                            escape_release_frames = 0
                    elif escape_phase == "push":
                        phase_elapsed = cycle_start - escape_phase_started_at
                        signed_displacement = (
                            0.0
                            if escape_start_ball is None
                            else (ball - escape_start_ball)
                            * escape_ball_direction
                        )
                        moving_evidence = (
                            filtered_velocity * escape_ball_direction
                            >= float(settings["escape_release_velocity_cm_s"])
                            or signed_displacement
                            >= float(
                                settings["escape_release_displacement_cm"]
                            )
                        )
                        escape_release_frames = (
                            escape_release_frames + 1
                            if moving_evidence
                            else 0
                        )
                        release_confirmed = escape_release_frames >= int(
                            settings["escape_release_confirm_frames"]
                        )
                        push_timed_out = phase_elapsed >= float(
                            settings["escape_push_max_s"]
                        )
                        if release_confirmed or push_timed_out:
                            # Once the ball breaks free, hand braking straight
                            # back to the cascade velocity loop.  Keeping the
                            # escape output active here caused the old
                            # push/brake state machine to fight the PID.
                            escape_phase = "cooldown"
                            escape_phase_started_at = cycle_start
                            escape_cooldown_until = (
                                cycle_start
                                + float(settings["escape_cooldown_s"])
                            )
                            escape_start_ball = None
                            escape_release_frames = 0
                    elif escape_phase == "brake":
                        phase_elapsed = cycle_start - escape_phase_started_at
                        brake_finished = (
                            phase_elapsed
                            >= float(settings["escape_recovery_min_s"])
                            and abs(filtered_velocity)
                            <= float(
                                settings["escape_recovery_velocity_cm_s"]
                            )
                        )
                        brake_timed_out = phase_elapsed >= float(
                            settings["escape_recovery_max_s"]
                        )
                        if brake_finished or brake_timed_out:
                            bumpless_output_steps = float(
                                last_escape_offset_steps
                            )
                            escape_phase = "cooldown"
                            escape_phase_started_at = cycle_start
                            escape_cooldown_until = (
                                cycle_start
                                + float(settings["escape_cooldown_s"])
                            )
                            escape_start_ball = None
                            escape_release_frames = 0

                    trigger_escape = False
                    if escape_phase == "idle" and manual_escape_requested:
                        trigger_escape = True

                    # A low-speed steady error inside the configured bias zone
                    # is not treated as a jam yet.  Let the bounded trim keep
                    # adding the missing static tilt; only hand over to the
                    # escape state machine after that trim is nearly exhausted.
                    bias_has_room = (
                        int(settings["control_mode"]) == 1
                        and int(settings["bias_enabled"])
                        and abs(position_error)
                        <= float(settings["bias_zone_cm"])
                        and abs(bias_steps)
                        < 0.95 * float(settings["bias_limit_steps"])
                    )

                    if (
                        escape_phase == "idle"
                        and int(settings["escape_enabled"])
                        and cycle_start >= escape_cooldown_until
                        and stationary_condition
                        and not bias_has_room
                    ):
                        if stationary_since is None:
                            stationary_since = cycle_start
                        stationary_elapsed_s = cycle_start - stationary_since
                        if stationary_elapsed_s >= float(
                            settings["escape_stationary_s"]
                        ):
                            trigger_escape = True
                    elif escape_phase == "idle" and not trigger_escape:
                        stationary_since = None
                    else:
                        stationary_since = None

                    if trigger_escape:
                        escape_ball_direction = (
                            1 if position_error >= 0.0 else -1
                        )
                        escape_axis_direction = (
                            escape_ball_direction
                            * int(settings["control_direction"])
                            * int(settings["escape_direction"])
                        )
                        escape_phase = (
                            "preload"
                            if int(settings["escape_preload_steps"]) > 0
                            and float(settings["escape_preload_s"]) > 0.0
                            else "push"
                        )
                        escape_phase_started_at = cycle_start
                        escape_start_ball = (
                            ball if escape_phase == "push" else None
                        )
                        escape_release_frames = 0
                        escape_count += 1
                        stationary_since = None
                        stationary_elapsed_s = 0.0

                    integrator_frozen = (
                        escape_phase in ("preload", "push", "brake")
                        or drive_feedforward_active
                    )

                    if escape_phase == "preload":
                        escape_active = True
                        escape_offset_steps = int(
                            round(
                                -escape_axis_direction
                                * int(settings["escape_preload_steps"])
                            )
                        )
                    elif escape_phase == "push":
                        escape_active = True
                        phase_elapsed = cycle_start - escape_phase_started_at
                        push_magnitude = clamp(
                            float(settings["escape_start_steps"])
                            + float(settings["escape_ramp_steps_per_s"])
                            * phase_elapsed,
                            float(settings["escape_start_steps"]),
                            float(settings["escape_push_steps"]),
                        )
                        escape_offset_steps = int(
                            round(escape_axis_direction * push_magnitude)
                        )
                    elif escape_phase == "brake":
                        escape_active = True
                        phase_elapsed = cycle_start - escape_phase_started_at
                        brake_floor_fade_s = max(
                            0.05,
                            float(settings["escape_recovery_min_s"]),
                        )
                        brake_floor = float(
                            settings["escape_brake_min_steps"]
                        ) * max(
                            0.0,
                            1.0 - phase_elapsed / brake_floor_fade_s,
                        )
                        brake_magnitude = clamp(
                            brake_floor
                            + float(
                                settings[
                                    "escape_brake_gain_steps_per_cm_s"
                                ]
                            )
                            * abs(filtered_velocity),
                            0.0,
                            float(settings["escape_recovery_steps"]),
                        )
                        brake_axis_direction = (
                            -escape_brake_ball_direction
                            * int(settings["control_direction"])
                            * int(settings["escape_direction"])
                        )
                        escape_offset_steps = int(
                            round(brake_axis_direction * brake_magnitude)
                        )

                    if escape_active:
                        last_escape_offset_steps = escape_offset_steps

                    effective_error = (
                        0.0
                        if abs(position_error) <= float(settings["deadband_cm"])
                        else position_error
                    )
                    if bumpless_output_steps is not None:
                        position_integral = 0.0

                    pos_integral_zone = float(
                        settings["pos_integral_zone_cm"]
                    )
                    position_integration_allowed = (
                        not integrator_frozen
                        and effective_error != 0.0
                        and (
                            pos_integral_zone <= 0.0
                            or abs(effective_error) <= pos_integral_zone
                        )
                    )
                    if integrator_frozen:
                        candidate_pos_integral = position_integral
                    elif effective_error == 0.0:
                        candidate_pos_integral = position_integral * 0.96
                    elif not position_integration_allowed:
                        candidate_pos_integral = position_integral * max(
                            0.0, 1.0 - dt
                        )
                    else:
                        candidate_pos_integral = clamp(
                            position_integral + effective_error * dt,
                            -float(settings["pos_integral_limit"]),
                            float(settings["pos_integral_limit"]),
                        )

                    outer_unsaturated = (
                        float(settings["pos_kp"]) * effective_error
                        + float(settings["pos_ki"]) * candidate_pos_integral
                        - float(settings["pos_kd"]) * filtered_velocity
                    )
                    max_target_velocity = float(
                        settings["max_target_velocity_cm_s"]
                    )
                    target_velocity = clamp(
                        outer_unsaturated,
                        -max_target_velocity,
                        max_target_velocity,
                    )
                    if not integrator_frozen and (
                        not position_integration_allowed
                        or outer_unsaturated == target_velocity
                        or outer_unsaturated * effective_error <= 0
                    ):
                        position_integral = candidate_pos_integral

                    velocity_error = target_velocity - filtered_velocity
                    derivative = 0.0
                    if previous_velocity_error is not None:
                        derivative = clamp(
                            (velocity_error - previous_velocity_error) / dt,
                            -150.0,
                            150.0,
                        )
                    previous_velocity_error = velocity_error

                    did_backcalculate = False
                    if bumpless_output_steps is not None:
                        desired_inner = (
                            bumpless_output_steps
                            / int(settings["control_direction"])
                        )
                        no_integral_inner = (
                            float(settings["vel_kp"]) * velocity_error
                            + float(settings["vel_kd"]) * derivative
                        )
                        vel_ki = float(settings["vel_ki"])
                        velocity_integral = (
                            clamp(
                                (desired_inner - no_integral_inner) / vel_ki,
                                -float(settings["vel_integral_limit"]),
                                float(settings["vel_integral_limit"]),
                            )
                            if vel_ki > 0.0
                            else 0.0
                        )
                        candidate_vel_integral = velocity_integral
                        bumpless_output_steps = None
                        did_backcalculate = True
                    elif integrator_frozen:
                        candidate_vel_integral = velocity_integral
                    else:
                        candidate_vel_integral = clamp(
                            velocity_integral + velocity_error * dt,
                            -float(settings["vel_integral_limit"]),
                            float(settings["vel_integral_limit"]),
                        )

                    stiction_feedforward_steps = 0.0
                    if (
                        int(settings["control_mode"]) == 0
                        and int(settings["stiction_ff_enabled"])
                        and effective_error != 0.0
                        and target_velocity * effective_error > 0.0
                    ):
                        target_direction = 1 if target_velocity > 0.0 else -1
                        progress_velocity = filtered_velocity * target_direction
                        fade_velocity = float(
                            settings["stiction_ff_fade_velocity_cm_s"]
                        )
                        feedforward_scale = 1.0 - clamp(
                            progress_velocity / fade_velocity,
                            0.0,
                            1.0,
                        )
                        feedforward_magnitude = float(
                            settings[
                                "stiction_ff_positive_steps"
                                if target_direction > 0
                                else "stiction_ff_negative_steps"
                            ]
                        )
                        stiction_feedforward_steps = (
                            target_direction
                            * feedforward_magnitude
                            * feedforward_scale
                        )

                    inner_unsaturated = (
                        float(settings["vel_kp"]) * velocity_error
                        + float(settings["vel_ki"]) * candidate_vel_integral
                        + float(settings["vel_kd"]) * derivative
                        + stiction_feedforward_steps
                    )
                    max_offset = float(settings["max_offset_steps"])
                    limited_inner = clamp(
                        inner_unsaturated,
                        -max_offset,
                        max_offset,
                    )
                    if did_backcalculate or (
                        not integrator_frozen
                        and (
                            inner_unsaturated == limited_inner
                            or inner_unsaturated * velocity_error <= 0
                        )
                    ):
                        velocity_integral = candidate_vel_integral

                    pid_output_steps = (
                        int(settings["control_direction"]) * limited_inner
                    )

                    if int(settings["control_mode"]) == 1:
                        # The recommended controller has no continuously
                        # accumulating position/velocity integral.  A small,
                        # slow bias is learned only near the setpoint and only
                        # while the ball is moving slowly without being stuck.
                        position_integral = 0.0
                        velocity_integral = 0.0
                        previous_velocity_error = None
                        target_velocity = 0.0
                        velocity_error = None

                        if effective_error < 0.0:
                            pd_direction_gain = float(
                                settings["pd_left_gain"]
                            )
                        elif effective_error > 0.0:
                            pd_direction_gain = float(
                                settings["pd_right_gain"]
                            )
                        pd_p_steps = (
                            float(settings["pd_kp_steps_per_cm"])
                            * effective_error
                            * pd_direction_gain
                        )
                        pd_d_steps = -(
                            float(settings["pd_kd_steps_per_cm_s"])
                            * filtered_velocity
                        )
                        bias_learning = (
                            bool(int(settings["bias_enabled"]))
                            and not integrator_frozen
                            and effective_error != 0.0
                            and abs(effective_error)
                            <= float(settings["bias_zone_cm"])
                            and abs(filtered_velocity)
                            <= float(settings["bias_velocity_limit_cm_s"])
                        )
                        bias_frozen = not bias_learning
                        candidate_bias = bias_steps
                        if bias_learning:
                            candidate_bias = clamp(
                                bias_steps
                                + float(settings["bias_ki_steps_per_cm_s"])
                                * effective_error
                                * dt,
                                -float(settings["bias_limit_steps"]),
                                float(settings["bias_limit_steps"]),
                            )

                        pd_unsaturated = (
                            pd_p_steps + pd_d_steps + candidate_bias
                        )
                        pd_limited = clamp(
                            pd_unsaturated,
                            -float(settings["max_offset_steps"]),
                            float(settings["max_offset_steps"]),
                        )
                        if bias_learning and (
                            pd_unsaturated == pd_limited
                            or pd_unsaturated * effective_error <= 0.0
                        ):
                            bias_steps = candidate_bias
                        if not int(settings["bias_enabled"]):
                            bias_steps = 0.0

                        # Re-evaluate with the accepted bias so a rejected
                        # anti-windup update never leaks into this command.
                        pd_output_steps = clamp(
                            pd_p_steps + pd_d_steps + bias_steps,
                            -float(settings["max_offset_steps"]),
                            float(settings["max_offset_steps"]),
                        )
                        pid_output_steps = (
                            int(settings["control_direction"])
                            * pd_output_steps
                        )
                    else:
                        bias_steps = 0.0

                    output_steps = (
                        escape_offset_steps
                        if escape_active
                        else pid_output_steps
                    )
                    output_steps += drive_feedforward_steps
                    desired = int(
                        round(
                            clamp(
                                int(settings["control_center_steps"])
                                + output_steps,
                                int(settings["control_min_steps"]),
                                int(settings["control_max_steps"]),
                            )
                        )
                    )
                    try:
                        self.state.set_control_target(desired, revision)
                    except RuntimeError as error:
                        self.state.disable(str(error))
                else:
                    stationary_since = None
                    escape_phase = "idle"
                    escape_phase_started_at = cycle_start
                    escape_start_ball = None
                    escape_release_frames = 0
                    last_escape_offset_steps = 0
                    bumpless_output_steps = None
            else:
                stationary_since = None
                escape_phase = "idle"
                escape_phase_started_at = cycle_start
                escape_start_ball = None
                escape_release_frames = 0
                last_escape_offset_steps = 0
                bumpless_output_steps = None

            vision_age = (
                max(0.0, time.time() - vision_timestamp)
                if vision_timestamp is not None
                else None
            )
            self.state.set_runtime(
                {
                    "ball_position_cm": ball,
                    "position_error_cm": position_error,
                    "velocity_cm_s": (
                        filtered_velocity if ball is not None else None
                    ),
                    "target_velocity_cm_s": target_velocity,
                    "velocity_error_cm_s": velocity_error,
                    "position_integral": round(position_integral, 4),
                    "velocity_integral": round(velocity_integral, 4),
                    "pd_p_steps": round(pd_p_steps, 2),
                    "pd_d_steps": round(pd_d_steps, 2),
                    "pd_output_steps": round(pd_output_steps, 2),
                    "pd_direction_gain": round(pd_direction_gain, 3),
                    "bias_steps": round(bias_steps, 3),
                    "bias_learning": bias_learning,
                    "bias_frozen": bias_frozen,
                    "control_output_steps": round(output_steps, 2),
                    "pid_output_steps": round(pid_output_steps, 2),
                    "stiction_feedforward_steps": round(
                        stiction_feedforward_steps, 2
                    ),
                    "escape_active": escape_active,
                    "escape_phase": escape_phase,
                    "escape_offset_steps": escape_offset_steps,
                    "escape_count": escape_count,
                    "integrator_frozen": integrator_frozen,
                    "escape_release_frames": escape_release_frames,
                    "stationary_elapsed_s": round(stationary_elapsed_s, 3),
                    "vision_fps": vision_fps,
                    "vision_age_s": vision_age,
                }
            )
            interval = 1.0 / float(settings["control_hz"])
            self.stop_event.wait(
                max(0.003, interval - (time.monotonic() - cycle_start))
            )


class HardwareButtons:
    """Four debounced, active-low buttons plus the drive-state input."""

    BUTTONS = (
        (BUTTON_EMERGENCY, "emergency"),
        (BUTTON_REFERENCE_ZERO, "reference"),
        (BUTTON_ENABLE_ZERO, "zero_loop"),
        (BUTTON_RUN_TRAJECTORY, "trajectory"),
    )

    def __init__(
        self,
        state: CascadeState,
        axis: DirectStepperAxis,
        shake_test: StepperShakeTest,
        trajectory_test: ScoredTrajectoryTest,
    ) -> None:
        self.state = state
        self.axis = axis
        self.shake_test = shake_test
        self.trajectory_test = trajectory_test
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="d36a-hardware-buttons",
            daemon=True,
        )
        self.levels = {name: 1 for _pin, name in self.BUTTONS}
        self.last_action = "待机"
        self.last_error = ""
        self.event_count = 0
        claimed: list[int] = []
        try:
            for pin, _name in self.BUTTONS:
                lgpio.gpio_claim_input(
                    self.axis.handle,
                    pin,
                    lgpio.SET_PULL_UP,
                )
                claimed.append(pin)
            lgpio.gpio_claim_input(
                self.axis.handle,
                DRIVE_SIGNAL,
                lgpio.SET_PULL_DOWN,
            )
            claimed.append(DRIVE_SIGNAL)
        except Exception:
            for pin in claimed:
                try:
                    lgpio.gpio_free(self.axis.handle, pin)
                except Exception:
                    pass
            raise

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "button_emergency_level": self.levels["emergency"],
                "button_reference_level": self.levels["reference"],
                "button_zero_loop_level": self.levels["zero_loop"],
                "button_trajectory_level": self.levels["trajectory"],
                "hardware_button_event_count": self.event_count,
                "hardware_button_last_action": self.last_action,
                "hardware_button_last_error": self.last_error,
            }

    def _cancel_tests(self, wait: bool) -> None:
        self.shake_test.cancel_immediately()
        self.trajectory_test.cancel_immediately()
        if not wait:
            return
        deadline = time.monotonic() + 0.60
        while time.monotonic() < deadline:
            if (
                not self.shake_test.is_active()
                and not self.trajectory_test.is_active()
            ):
                return
            self.stop_event.wait(0.01)

    def _dispatch(self, pin: int, name: str) -> None:
        action = ""
        try:
            if name == "emergency":
                self._cancel_tests(False)
                self.state.disable(
                    "硬件急停按键：STEP已停止、EN已释放"
                )
                self.axis.emergency_stop()
                action = "BCM5：紧急停止并释放驱动"
            elif name == "reference":
                self._cancel_tests(True)
                self.state.disable("")
                self.axis.reference_zero()
                action = "BCM6：当前位置已确认为最低点0"
            elif name == "zero_loop":
                self._cancel_tests(True)
                self.state.set_target(0.0)
                self.state.enable()
                action = "BCM16：目标0 cm闭环已开启"
            elif name == "trajectory":
                if self.shake_test.is_active():
                    raise RuntimeError("抖动测试仍在运行")
                self.trajectory_test.start()
                action = "BCM20：开环→-5 cm闭环题目已启动"
            else:
                raise RuntimeError(f"未知硬件按键{pin}")
            with self.lock:
                self.event_count += 1
                self.last_action = action
                self.last_error = ""
            print(f"Hardware button: {action}", flush=True)
        except (RuntimeError, OSError, ValueError) as error:
            message = f"BCM{pin}按键未执行：{error}"
            with self.lock:
                self.event_count += 1
                self.last_error = message
            print(f"Hardware button error: {message}", flush=True)

    def _run(self) -> None:
        now = time.monotonic()
        pin_state: dict[int, dict[str, object]] = {}
        for pin, name in self.BUTTONS:
            raw = int(lgpio.gpio_read(self.axis.handle, pin))
            with self.lock:
                self.levels[name] = raw
            pin_state[pin] = {
                "name": name,
                "raw": raw,
                "stable": 1,
                "changed_at": now,
                # Non-emergency buttons must first be observed released.  This
                # avoids a held button setting a false zero during service boot.
                "armed": raw == 1 or name == "emergency",
            }
        drive_raw = int(lgpio.gpio_read(self.axis.handle, DRIVE_SIGNAL))
        drive_state: dict[str, object] = {
            "raw": drive_raw,
            "stable": drive_raw,
            "changed_at": now,
        }
        self.state.set_runtime({"drive_signal_level": drive_raw})

        while not self.stop_event.wait(0.01):
            now = time.monotonic()
            for pin, name in self.BUTTONS:
                state = pin_state[pin]
                try:
                    raw = int(lgpio.gpio_read(self.axis.handle, pin))
                except Exception as error:
                    with self.lock:
                        self.last_error = f"BCM{pin}读取失败：{error}"
                    continue
                with self.lock:
                    self.levels[name] = raw
                if raw != state["raw"]:
                    state["raw"] = raw
                    state["changed_at"] = now
                    continue
                if (
                    raw == state["stable"]
                    or now - float(state["changed_at"]) < BUTTON_DEBOUNCE_S
                ):
                    continue
                state["stable"] = raw
                if raw == 1:
                    state["armed"] = True
                elif bool(state["armed"]):
                    state["armed"] = False
                    self._dispatch(pin, name)
            try:
                drive_raw = int(
                    lgpio.gpio_read(self.axis.handle, DRIVE_SIGNAL)
                )
            except Exception as error:
                with self.lock:
                    self.last_error = f"BCM26发车信号读取失败：{error}"
                continue
            if drive_raw != drive_state["raw"]:
                drive_state["raw"] = drive_raw
                drive_state["changed_at"] = now
            elif (
                drive_raw != drive_state["stable"]
                and now - float(drive_state["changed_at"])
                >= DRIVE_SIGNAL_DEBOUNCE_S
            ):
                drive_state["stable"] = drive_raw
                self.state.trigger_drive_signal(drive_raw)
                direction = "向右发车" if drive_raw else "向左停车"
                print(
                    f"Drive signal BCM26: {direction}",
                    flush=True,
                )

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)
        for pin, _name in self.BUTTONS:
            try:
                lgpio.gpio_free(self.axis.handle, pin)
            except Exception:
                pass
        try:
            lgpio.gpio_free(self.axis.handle, DRIVE_SIGNAL)
        except Exception:
            pass


class SerialBridge:
    """Reconnectable UART4 line receiver and web-controlled transmitter."""

    PARAMETER_NAMES = (
        "VKP",
        "VKI",
        "TKP",
        "TKD",
        "P1_MAX",
        "P2_MAX",
        "AB_MAX",
        "P1_STOP",
        "P2_STOP",
        "AB_STOP",
        "CENTER",
        "DEAD",
        "MAX_TURN",
    )
    PARAMETER_PATTERN = re.compile(
        r"(?<![A-Z0-9_])("
        + "|".join(re.escape(name) for name in PARAMETER_NAMES)
        + r")(?![A-Z0-9_])\s*(?:[=:,]\s*|\s+)"
        + r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
        re.IGNORECASE,
    )

    def __init__(
        self,
        port: str = "/dev/ttyAMA4",
        baudrate: int = 115200,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="stm32-uart4-bridge",
            daemon=True,
        )
        self.serial_port: serial.Serial | None = None
        self.connected = False
        self.error = ""
        self.rx_lines: deque[str] = deque(maxlen=120)
        self.rx_buffer = bytearray()
        self.rx_bytes = 0
        self.rx_lines_count = 0
        self.last_rx_chunk_at = 0.0
        self.tx_bytes = 0
        self.tx_messages = 0
        self.last_rx = ""
        self.last_tx = ""
        self.parameters: dict[str, str] = {}

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "serial_port": self.port,
                "serial_baudrate": self.baudrate,
                "serial_connected": self.connected,
                "serial_error": self.error,
                "serial_rx_bytes": self.rx_bytes,
                "serial_rx_lines_count": self.rx_lines_count,
                "serial_tx_bytes": self.tx_bytes,
                "serial_tx_messages": self.tx_messages,
                "serial_last_rx": self.last_rx,
                "serial_last_tx": self.last_tx,
                "serial_rx_lines": list(self.rx_lines),
                "serial_parameters": dict(self.parameters),
            }

    def _record_rx_line(self, raw: bytes) -> None:
        text = raw.rstrip(b"\r").decode("utf-8", errors="replace")
        timestamp = time.strftime("%H:%M:%S")
        display = f"{timestamp} RX  {text}"
        with self.lock:
            self.rx_lines.append(display)
            self.rx_lines_count += 1
            self.last_rx = text
            for match in self.PARAMETER_PATTERN.finditer(text):
                self.parameters[match.group(1).upper()] = match.group(2)

    def _close_port(self) -> None:
        with self.lock:
            port = self.serial_port
            self.serial_port = None
            self.connected = False
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def _connect(self) -> None:
        port = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.10,
            write_timeout=0.50,
            exclusive=True,
        )
        with self.lock:
            self.serial_port = port
            self.connected = True
            self.error = ""

    def _run(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                port = self.serial_port
            if port is None:
                try:
                    self._connect()
                except (OSError, serial.SerialException) as error:
                    with self.lock:
                        self.error = f"{self.port}连接失败：{error}"
                    self.stop_event.wait(1.0)
                    continue
                with self.lock:
                    port = self.serial_port
            try:
                assert port is not None
                chunk = port.read(max(1, port.in_waiting))
                if not chunk:
                    raw = b""
                    with self.lock:
                        if (
                            self.rx_buffer
                            and time.monotonic() - self.last_rx_chunk_at
                            >= 0.20
                        ):
                            raw = bytes(self.rx_buffer)
                            self.rx_buffer.clear()
                    if raw:
                        self._record_rx_line(raw)
                    continue
                with self.lock:
                    self.rx_bytes += len(chunk)
                    self.last_rx_chunk_at = time.monotonic()
                    self.rx_buffer.extend(chunk)
                    while b"\n" in self.rx_buffer:
                        raw, _, remaining = self.rx_buffer.partition(b"\n")
                        self.rx_buffer = bytearray(remaining)
                        self._record_rx_line(raw)
                    if len(self.rx_buffer) >= 2048:
                        raw = bytes(self.rx_buffer)
                        self.rx_buffer.clear()
                        self._record_rx_line(raw)
            except (OSError, serial.SerialException) as error:
                with self.lock:
                    self.error = f"串口读取失败：{error}"
                self._close_port()
                self.stop_event.wait(0.50)

    def send(self, data: str, append_newline: bool) -> int:
        if not isinstance(data, str):
            raise ValueError("串口发送数据必须是文本")
        encoded = data.encode("utf-8")
        if append_newline:
            encoded += b"\n"
        if not encoded:
            raise ValueError("串口发送内容不能为空")
        if len(encoded) > 4096:
            raise ValueError("单次串口发送不能超过4096字节")
        with self.write_lock:
            with self.lock:
                port = self.serial_port
                connected = self.connected
            if port is None or not connected:
                raise RuntimeError(self.error or "UART4尚未连接")
            try:
                written = port.write(encoded)
                port.flush()
            except (OSError, serial.SerialException) as error:
                self._close_port()
                raise RuntimeError(f"串口发送失败：{error}") from error
        with self.lock:
            self.tx_bytes += written
            self.tx_messages += 1
            self.last_tx = data
            self.rx_lines.append(
                f"{time.strftime('%H:%M:%S')} TX  {data}"
            )
        return written

    def clear_log(self) -> None:
        with self.lock:
            self.rx_lines.clear()
            self.last_rx = ""

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)
        self._close_port()


class ControlServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        state: CascadeState,
        axis: DirectStepperAxis,
        shake_test: StepperShakeTest,
        trajectory_test: ScoredTrajectoryTest,
        hardware_buttons: HardwareButtons,
        serial_bridge: SerialBridge,
    ) -> None:
        self.state = state
        self.axis = axis
        self.shake_test = shake_test
        self.trajectory_test = trajectory_test
        self.hardware_buttons = hardware_buttons
        self.serial_bridge = serial_bridge
        super().__init__(address, ControlHandler)


class ControlHandler(BaseHTTPRequestHandler):
    server: ControlServer

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def send_bytes(
        self,
        payload: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def send_json(
        self,
        payload: dict[str, object],
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_bytes(
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            "application/json; charset=utf-8",
            status,
        )

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/":
            self.send_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/status.json":
            payload = self.server.state.snapshot()
            payload.update(self.server.shake_test.snapshot())
            payload.update(self.server.trajectory_test.snapshot())
            payload.update(self.server.hardware_buttons.snapshot())
            payload.update(self.server.serial_bridge.snapshot())
            self.send_json(payload)
        else:
            self.send_bytes(
                b"Not found", "text/plain", HTTPStatus.NOT_FOUND
            )

    def do_POST(self) -> None:
        if urlsplit(self.path).path != "/control":
            self.send_bytes(
                b"Not found", "text/plain", HTTPStatus.NOT_FOUND
            )
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("请求大小无效")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("JSON必须是对象")
            action = payload.get("action")
            if (
                self.server.shake_test.is_active()
                and action not in (
                    "shake_stop",
                    "emergency_stop",
                    "serial_send",
                    "serial_clear",
                )
            ):
                raise RuntimeError(
                    "抖动测试运行中，请先停止测试或使用紧急停止"
                )
            if (
                self.server.trajectory_test.is_active()
                and action not in (
                    "trajectory_stop",
                    "emergency_stop",
                    "serial_send",
                    "serial_clear",
                )
            ):
                raise RuntimeError(
                    "一键比赛轨迹运行中，请先停止轨迹或使用紧急停止"
                )
            if action == "reference_zero":
                self.server.state.disable("")
                self.server.axis.reference_zero()
                message = "当前位置已设为最低点0，轴已恢复保持"
            elif action == "resume_axis":
                self.server.axis.resume()
                message = "轴已恢复保持，可再次启动闭环"
            elif action == "emergency_stop":
                self.server.shake_test.cancel_immediately()
                self.server.trajectory_test.cancel_immediately()
                self.server.state.disable(
                    "紧急停止：STEP已停、EN已释放；网页仍可恢复轴"
                )
                self.server.axis.emergency_stop()
                message = "紧急停止完成，网页和轴对象仍在"
            elif action == "enable":
                self.server.state.enable()
                message = "闭环控制已开启"
            elif action == "disable":
                self.server.state.disable("")
                message = "闭环已停止，保持当前位置"
            elif action == "return_balance":
                center = self.server.state.control_center()
                self.server.state.disable(f"正在回控制中心{center}")
                self.server.axis.set_target(center)
                message = f"已开始回控制中心{center}"
            elif action == "test_escape":
                self.server.state.request_escape()
                message = "已触发一次朝目标大角度脱困"
            elif action == "shake_start":
                result = self.server.shake_test.start(payload)
                if result["mode"] == "fixed":
                    message = (
                        "固定频率抖动测试已开始："
                        f"中心{result['center_steps']}步，"
                        f"幅度±{result['amplitude_steps']}步"
                    )
                else:
                    message = (
                        "逐档扫频已开始："
                        f"共{result['stage_count']}档，"
                        f"中心{result['center_steps']}步，"
                        f"幅度±{result['amplitude_steps']}步"
                    )
            elif action == "shake_stop":
                stopped = self.server.shake_test.stop()
                message = (
                    "正在停止测试并回到测试中心"
                    if stopped
                    else "当前没有运行抖动测试"
                )
            elif action == "trajectory_start":
                if self.server.shake_test.is_active():
                    raise RuntimeError("请先停止抖动测试")
                self.server.trajectory_test.start()
                message = "控制键已接收：立即执行向右开环，随后自动切换-5 cm闭环"
            elif action == "trajectory_stop":
                stopped = self.server.trajectory_test.stop()
                message = (
                    "正在停止本题并保持当前位置"
                    if stopped
                    else "当前没有运行本题"
                )
            elif action == "drive_feedforward_test":
                level = int(payload.get("level"))
                if level not in (0, 1):
                    raise ValueError("前馈测试level只能是0或1")
                if not self.server.state.snapshot()["control_enabled"]:
                    raise RuntimeError("请先开启小球位置闭环再测试前馈")
                self.server.state.trigger_drive_signal(level)
                message = (
                    "已触发向右发车前馈"
                    if level
                    else "已触发向左停车前馈"
                )
            elif action == "serial_send":
                written = self.server.serial_bridge.send(
                    payload.get("data"),
                    bool(payload.get("append_newline", True)),
                )
                message = f"已向STM32发送{written}字节"
            elif action == "serial_clear":
                self.server.serial_bridge.clear_log()
                message = "网页串口接收记录已清空"
            elif action == "set_target":
                value = float(payload.get("target_cm"))
                self.server.state.set_target(value)
                if bool(payload.get("enable", False)):
                    self.server.state.enable()
                    message = f"目标位置已设为{value:+g} cm，单环PD已开启"
                else:
                    message = f"目标位置已设为{value:+g} cm"
            elif action == "move":
                self.server.state.disable("")
                value = int(payload.get("steps"))
                if value == 0 or abs(value) > 50:
                    raise ValueError("单次移动必须在-50～+50步且不能为0")
                axis = self.server.axis.snapshot()
                self.server.axis.set_target(
                    int(axis["current_steps"]) + value
                )
                message = f"手动移动{value:+d}步"
            elif action == "goto":
                self.server.state.disable("")
                value = int(payload.get("position"))
                if not 0 <= value <= 681:
                    raise ValueError("绝对位置必须在0～681")
                self.server.axis.set_target(value)
                message = f"移动到绝对位置{value}"
            elif action == "settings":
                values = payload.get("settings")
                if not isinstance(values, dict):
                    raise ValueError("settings必须是对象")
                self.server.state.update_settings(
                    values,
                    bool(payload.get("persist", False)),
                )
                message = (
                    "闭环参数已保存"
                    if payload.get("persist")
                    else "闭环参数已应用"
                )
            else:
                raise ValueError("不支持的控制命令")
            self.send_json({"ok": True, "message": message})
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except (RuntimeError, OSError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.CONFLICT)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--minimum", type=int, default=0)
    parser.add_argument("--balance", type=int, default=235)
    parser.add_argument("--maximum", type=int, default=681)
    parser.add_argument("--up-dir-level", type=int, choices=(0, 1), default=1)
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path(
            "/home/arick/d36a_test/cascade_stepper_settings.json"
        ),
    )
    parser.add_argument(
        "--vision-status-url",
        default="http://127.0.0.1:8080/status.json",
    )
    parser.add_argument("--serial-port", default="/dev/ttyAMA4")
    parser.add_argument("--serial-baudrate", type=int, default=115200)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    axis = DirectStepperAxis(
        args.minimum,
        args.balance,
        args.maximum,
        args.up_dir_level,
    )
    state = CascadeState(args.settings, axis)
    controller = CascadeController(state, axis, args.vision_status_url)
    shake_test = StepperShakeTest(state, axis)
    trajectory_test = ScoredTrajectoryTest(state, axis)
    hardware_buttons = HardwareButtons(
        state,
        axis,
        shake_test,
        trajectory_test,
    )
    serial_bridge = SerialBridge(args.serial_port, args.serial_baudrate)
    server = ControlServer(
        (args.host, args.port),
        state,
        axis,
        shake_test,
        trajectory_test,
        hardware_buttons,
        serial_bridge,
    )
    controller.start()
    hardware_buttons.start()
    serial_bridge.start()

    def stop_signal(_signum: int, _frame: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    print(
        f"Direct ball controller: http://{args.host}:{args.port}/ "
        "(unreferenced, closed loop disabled)",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        serial_bridge.close()
        hardware_buttons.close()
        trajectory_test.close()
        shake_test.close()
        state.disable("控制程序已停止")
        controller.stop()
        server.server_close()
        axis.close()


if __name__ == "__main__":
    main()
