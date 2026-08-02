"""Live path simulator for Electric_Vehicle drive logic.

This script mirrors the control structure in src/main.cpp and animates
the trajectory in real time so tuning changes are immediately visible.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field
from typing import List

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class SimConfig:
    target_distance_m: float = 10.489
    distance_scale: float = 0.952  # Scale multiplier (-0.5 m reduction)
    front_to_wheel_center_m: float = 0.109
    target_distance_is_front_reference: bool = True
    stop_extra_m: float = 0.470  # Explicit +470 mm extra travel at stop condition
    end_lateral_bias_m: float = 0.01  # Additional +7 cm left trim from prior tuning
    use_time_scaling: bool = True
    target_run_time_s: float = 14.0  # Clamped to 10-20s and snapped to 0.5s
    time_target_min_s: float = 10.0
    time_target_max_s: float = 20.0
    time_target_step_s: float = 0.5
    time_scale_min: float = 0.88
    time_scale_max: float = 1.18
    time_scale_kp: float = 0.85
    max_safe_power: float = 0.48
    use_arc: bool = True
    arc_max_angle_deg: float = 7.0  # Rolled back slightly for stability
    arc_target_offset_m: float = 0.86  # Rolled back slightly for stability
    arc_lateral_shape_exp: float = 1.0
    drive_side: int = 1

    wheel_diam_m: float = 0.0525
    wheelbase_m: float = 0.18
    dt_s: float = 0.02

    base_power: float = 0.35
    min_power: float = 0.10
    # Accel & decel as fractions of effective target distance (reliable scaling across 7-10m range)
    accel_frac: float = 0.0664  # ~6.64% of distance for smooth launch
    decel_frac: float = 0.1881  # ~18.81% of distance for smooth stop
    accel_dist_m: float = 0.60  # Will be computed per-run
    decel_dist_m: float = 1.70  # Will be computed per-run
    end_creep_zone_m: float = 0.40
    end_creep_min_power: float = 0.125  # Balanced end-of-run minimum torque

    sm_gain: float = 0.045
    xt_gain: float = -45.0
    ki_lateral: float = 0.009
    drive_trim: float = 0.016

    end_center_start: float = 0.52
    end_heading_damp: float = 0.18
    end_center_boost: float = 4.2

    max_wheel_speed_mps: float = 0.55
    left_scale: float = 1.00
    right_scale: float = 1.00
    wheel_speed_noise_mps: float = 0.0

    seed: int = 11

    @property
    def meters_per_deg(self) -> float:
        return (math.pi * self.wheel_diam_m) / 360.0


@dataclass
class SimState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 0.0
    max_heading_deg: float = 0.0
    forward_x_m: float = 0.0
    lateral_y_m: float = 0.0

    cum1_deg: float = 0.0
    cum2_deg: float = 0.0
    lateral_integral: float = 0.0
    filtered_speed_error: float = 0.0

    t_s: float = 0.0
    step_count: int = 0
    done: bool = False
    last_step_left_deg: float = 0.0
    last_step_right_deg: float = 0.0


@dataclass
class Trace:
    t_s: List[float] = field(default_factory=list)
    x_m: List[float] = field(default_factory=lambda: [0.0])
    y_m: List[float] = field(default_factory=lambda: [0.0])
    heading_deg: List[float] = field(default_factory=lambda: [0.0])
    target_heading_deg: List[float] = field(default_factory=lambda: [0.0])
    left_power: List[float] = field(default_factory=lambda: [0.0])
    right_power: List[float] = field(default_factory=lambda: [0.0])


class ElectricVehicleSimulator:
    def __init__(self, cfg: SimConfig) -> None:
        self.cfg = cfg
        self.state = SimState()
        self.trace = Trace()
        self.rng = random.Random(cfg.seed)

    def _sanitize_target_time_s(self) -> float:
        clamped = clamp(self.cfg.target_run_time_s, self.cfg.time_target_min_s, self.cfg.time_target_max_s)
        snapped = round(clamped / self.cfg.time_target_step_s) * self.cfg.time_target_step_s
        return clamp(snapped, self.cfg.time_target_min_s, self.cfg.time_target_max_s)

    def _compute_arc_heading(self, progress: float) -> float:
        wave = math.sin(progress * 2.0 * math.pi)
        envelope = math.sin(progress * math.pi)
        return_boost = 1.0
        if progress > 0.50:
            return_boost = 1.0 + 0.18 * ((progress - 0.50) / 0.50)
        return wave * envelope * self.cfg.arc_max_angle_deg * return_boost * self.cfg.drive_side

    def _compute_arc_target_lateral(self, progress: float) -> float:
        shaped_progress = clamp(progress, 0.0, 1.0) ** self.cfg.arc_lateral_shape_exp
        return self.cfg.drive_side * self.cfg.arc_target_offset_m * math.sin(shaped_progress * math.pi)

    def _effective_target_distance_m(self) -> float:
        target = self.cfg.target_distance_m * self.cfg.distance_scale
        if self.cfg.target_distance_is_front_reference:
            target -= self.cfg.front_to_wheel_center_m
        return max(0.05, target)

    def _record(self, target_heading_deg: float, left_power: float, right_power: float) -> None:
        s = self.state
        self.trace.t_s.append(s.t_s)
        self.trace.x_m.append(s.x_m)
        self.trace.y_m.append(s.y_m)
        self.trace.heading_deg.append(s.heading_deg)
        self.trace.target_heading_deg.append(target_heading_deg)
        self.trace.left_power.append(left_power)
        self.trace.right_power.append(right_power)

    def step(self) -> None:
        if self.state.done:
            return

        cfg = self.cfg
        s = self.state
        
        # Compute proportional accel/decel zones based on target distance for consistent behavior
        effective_target = self._effective_target_distance_m()
        cfg.accel_dist_m = effective_target * cfg.accel_frac
        cfg.decel_dist_m = effective_target * cfg.decel_frac

        target_distance_m = effective_target + cfg.stop_extra_m
        remaining = target_distance_m - s.forward_x_m
        if remaining <= 0.0:
            s.done = True
            self._record(0.0, 0.0, 0.0)
            return

        remaining_for_control = remaining if remaining > 0.0 else 0.0
        target_time_s = self._sanitize_target_time_s()

        progress = clamp(s.forward_x_m / target_distance_m, 0.0, 1.0)
        target_heading = 0.0
        lateral_ref = 0.0
        if cfg.use_arc:
            target_heading = self._compute_arc_heading(progress)
            lateral_ref = self._compute_arc_target_lateral(progress)
            end_bias_blend = clamp((progress - 0.72) / 0.28, 0.0, 1.0)
            lateral_ref += cfg.end_lateral_bias_m * end_bias_blend

            end_blend = clamp(
                (progress - cfg.end_center_start) / max(1e-6, (1.0 - cfg.end_center_start)),
                0.0,
                1.0,
            )
            target_heading *= (1.0 - end_blend)

        lateral_ctrl_error = (s.lateral_y_m - lateral_ref) if cfg.use_arc else s.lateral_y_m
        s.lateral_integral += lateral_ctrl_error * cfg.dt_s
        s.lateral_integral = clamp(s.lateral_integral, -0.35, 0.35)

        center_assist = 1.25 if progress > 0.66 else 1.0
        if cfg.use_arc and progress > 0.55:
            center_assist *= 1.15

        if cfg.use_arc and progress > cfg.end_center_start:
            end_blend = clamp(
                (progress - cfg.end_center_start) / max(1e-6, (1.0 - cfg.end_center_start)),
                0.0,
                1.0,
            )
            center_assist *= (1.0 + cfg.end_center_boost * end_blend)
            target_heading += -cfg.end_heading_damp * end_blend * s.heading_deg

        target_heading += center_assist * (
            cfg.xt_gain * lateral_ctrl_error + cfg.ki_lateral * s.lateral_integral
        )
        target_heading = clamp(target_heading, -40.0, 40.0)

        h_error = s.heading_deg - target_heading
        if abs(h_error) < 1.0:
            h_error = 0.0

        if s.forward_x_m < cfg.accel_dist_m:
            f = clamp(s.forward_x_m / cfg.accel_dist_m, 0.0, 1.0)
            current_power = cfg.min_power + f * (cfg.base_power - cfg.min_power)
        elif remaining_for_control < cfg.decel_dist_m:
            f = clamp(remaining_for_control / cfg.decel_dist_m, 0.0, 1.0)
            current_power = cfg.min_power + f * (cfg.base_power - cfg.min_power)
        else:
            current_power = cfg.base_power

        if remaining_for_control < cfg.end_creep_zone_m:
            f = clamp(remaining_for_control / cfg.end_creep_zone_m, 0.0, 1.0)
            current_power = cfg.end_creep_min_power + f * (current_power - cfg.end_creep_min_power)

        if cfg.use_time_scaling:
            desired_progress = clamp(s.t_s / target_time_s, 0.0, 1.0)
            progress_error = desired_progress - progress
            time_scale = clamp(1.0 + cfg.time_scale_kp * progress_error, cfg.time_scale_min, cfg.time_scale_max)
            current_power *= time_scale

        # Enforce minimum torque at the finish so the vehicle can still move.
        if current_power < cfg.end_creep_min_power:
            current_power = cfg.end_creep_min_power
        if current_power > cfg.max_safe_power:
            current_power = cfg.max_safe_power

        if remaining_for_control > 0.30:
            if current_power < 0.35:
                kg = 0.009
            elif current_power < 0.50:
                kg = 0.014
            else:
                kg = 0.019
            gyro_corr = kg * h_error
        else:
            gyro_corr = 0.010 * h_error

        gyro_corr = clamp(gyro_corr, -0.28, 0.28)

        speed_balance = 0.0
        if not cfg.use_arc:
            speed_error = s.last_step_left_deg - s.last_step_right_deg
            if abs(speed_error) < 0.5:
                speed_error = 0.0
            s.filtered_speed_error = 0.7 * s.filtered_speed_error + 0.3 * speed_error
            speed_balance = (
                cfg.sm_gain * s.filtered_speed_error
                if remaining_for_control > 0.30
                else 0.022 * s.filtered_speed_error
            )

        left_power = clamp(current_power - speed_balance - gyro_corr - cfg.drive_trim, -1.0, 1.0)
        right_power = clamp(current_power + speed_balance + gyro_corr + cfg.drive_trim, -1.0, 1.0)

        left_speed = (
            left_power * cfg.max_wheel_speed_mps * cfg.left_scale
            + self.rng.gauss(0.0, cfg.wheel_speed_noise_mps)
        )
        right_speed = (
            right_power * cfg.max_wheel_speed_mps * cfg.right_scale
            + self.rng.gauss(0.0, cfg.wheel_speed_noise_mps)
        )

        d_left_m = left_speed * cfg.dt_s
        d_right_m = right_speed * cfg.dt_s
        d_center_m = 0.5 * (d_left_m + d_right_m)

        # Sign matches firmware steering convention where right>left reduces heading.
        yaw_rate_deg = ((left_speed - right_speed) / cfg.wheelbase_m) * RAD_TO_DEG
        s.heading_deg += yaw_rate_deg * cfg.dt_s
        if abs(s.heading_deg) > abs(s.max_heading_deg):
            s.max_heading_deg = s.heading_deg

        heading_rad = s.heading_deg * DEG_TO_RAD
        s.forward_x_m += d_center_m * math.cos(heading_rad)
        s.lateral_y_m += d_center_m * math.sin(heading_rad)

        s.x_m = s.forward_x_m
        s.y_m = s.lateral_y_m

        s.last_step_left_deg = d_left_m / cfg.meters_per_deg
        s.last_step_right_deg = d_right_m / cfg.meters_per_deg
        s.cum1_deg += s.last_step_left_deg
        s.cum2_deg += s.last_step_right_deg

        s.t_s += cfg.dt_s
        s.step_count += 1

        self._record(target_heading, left_power, right_power)

    def run_live(self, speedup: float = 1.0) -> None:
        fig, ax = plt.subplots(figsize=(8.2, 6.0))
        ax.set_title("Electric Vehicle Live Path Simulator")
        ax.set_xlabel("Forward X (m)")
        ax.set_ylabel("Lateral Y (m)")
        ax.grid(alpha=0.3)

        trail, = ax.plot([], [], "-", linewidth=2.0, color="tab:blue", alpha=0.75)
        dot, = ax.plot([], [], "o", markersize=7, color="tab:red")
        heading_line, = ax.plot([], [], "-", linewidth=2.2, color="tab:orange")
        info = ax.text(
            0.02,
            0.98,
            "",
            transform=ax.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
        )

        target_distance_m = self._effective_target_distance_m()
        ax.set_xlim(-0.2, max(1.0, target_distance_m + 0.4))
        ypad = max(0.4, self.cfg.arc_target_offset_m + 0.5)
        ax.set_ylim(-ypad, ypad)

        speedup = max(0.1, speedup)
        steps_per_frame = max(1, int(round(speedup)))

        def update(_frame: int):
            if not self.state.done:
                for _ in range(steps_per_frame):
                    self.step()
                    if self.state.done:
                        break

            x_vals = self.trace.x_m
            y_vals = self.trace.y_m
            x = x_vals[-1]
            y = y_vals[-1]

            heading_rad = self.state.heading_deg * DEG_TO_RAD
            hx = x + 0.18 * math.cos(heading_rad)
            hy = y + 0.18 * math.sin(heading_rad)

            trail.set_data(x_vals, y_vals)
            dot.set_data([x], [y])
            heading_line.set_data([x, hx], [y, hy])

            info.set_text(
                f"t={self.state.t_s:5.2f}s\n"
                f"x={self.state.forward_x_m:5.3f} m\n"
                f"front_x={(self.state.forward_x_m + self.cfg.front_to_wheel_center_m):5.3f} m\n"
                f"y={self.state.lateral_y_m:+5.3f} m\n"
                f"heading={self.state.heading_deg:+6.2f} deg\n"
                f"max|heading|={abs(self.state.max_heading_deg):5.2f} deg"
            )

            # Keep the interesting section centered as the vehicle advances.
            x_pad = 0.5
            ax.set_xlim(-0.2, max(target_distance_m + 0.4, x + x_pad))

            if self.state.done:
                anim.event_source.stop()

            return trail, dot, heading_line, info

        max_frames = int(1 + (target_distance_m / (self.cfg.base_power * self.cfg.max_wheel_speed_mps * self.cfg.dt_s)) * 2.5)
        anim = FuncAnimation(
            fig,
            update,
            frames=max_frames,
            interval=max(1, int((self.cfg.dt_s * 1000) / speedup)),
            blit=False,
            repeat=False,
        )
        plt.show()


def parse_args() -> argparse.Namespace:
    defaults = SimConfig()
    parser = argparse.ArgumentParser(description="Live path simulator for Electric_Vehicle")
    parser.add_argument("--distance", type=float, default=defaults.target_distance_m, help="target distance in meters")
    parser.add_argument("--base-power", type=float, default=defaults.base_power, help="base drive power")
    parser.add_argument("--min-power", type=float, default=defaults.min_power, help="minimum drive power")
    parser.add_argument("--arc", action="store_true", help="enable S-curve arc logic")
    parser.add_argument("--no-arc", action="store_true", help="disable S-curve arc logic")
    parser.add_argument("--arc-max-angle", type=float, default=defaults.arc_max_angle_deg, help="max arc heading angle")
    parser.add_argument("--arc-offset", type=float, default=defaults.arc_target_offset_m, help="target midpoint lateral offset")
    parser.add_argument("--arc-shape-exp", type=float, default=defaults.arc_lateral_shape_exp, help="arc lateral shape exponent (1.0 peaks at half distance)")
    parser.add_argument("--drive-side", type=int, choices=[-1, 1], default=defaults.drive_side, help="1=left, -1=right")
    parser.add_argument("--dt", type=float, default=defaults.dt_s, help="simulation time step")
    parser.add_argument("--speedup", type=float, default=3.0, help="animation speed multiplier")
    parser.add_argument("--target-time", type=float, default=defaults.target_run_time_s, help="desired completion time in seconds")
    parser.add_argument("--no-time-scaling", action="store_true", help="disable time-target speed scaling")
    parser.add_argument("--seed", type=int, default=defaults.seed, help="noise seed")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    use_arc = True
    if args.no_arc:
        use_arc = False
    elif args.arc:
        use_arc = True

    cfg = SimConfig(
        target_distance_m=args.distance,
        base_power=args.base_power,
        min_power=args.min_power,
        use_arc=use_arc,
        arc_max_angle_deg=args.arc_max_angle,
        arc_target_offset_m=args.arc_offset,
        arc_lateral_shape_exp=args.arc_shape_exp,
        drive_side=args.drive_side,
        dt_s=args.dt,
        target_run_time_s=args.target_time,
        use_time_scaling=not args.no_time_scaling,
        seed=args.seed,
    )

    sim = ElectricVehicleSimulator(cfg)
    sim.run_live(speedup=args.speedup)


if __name__ == "__main__":
    main()