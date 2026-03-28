#!/usr/bin/env python3
"""
Lokaler JSON-Bridge-Server fuer P2 (TinkerForge-/ESP32-Messkette).

Stellt eine kleine HTTP-API bereit, damit die HTML-Oberflaeche
den code-nahen Messkettenzustand pollen und steuern kann.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "simulation_config.json"
CODE_PATH = ROOT / "src" / "main.cpp"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class P2Simulator:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._load_config()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _load_config(self) -> None:
        cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        self.motor_cfg = cfg.get("motor", {})
        self.brake_cfg = cfg.get("brake", {})
        self.p2_cfg = cfg.get("p2Hardware", {})
        self.measurement_cfg = cfg.get("measurement", {})
        self.numerics_cfg = cfg.get("numerics", {})
        self.thermal_cfg = cfg.get("thermal", {})
        self.magnet_cfg = cfg.get("magnet", {})
        self.materials_cfg = cfg.get("materials", {})

        self.max_rpm = float(self.motor_cfg.get("maxRPM", 3000.0))
        self.max_power_w = float(self.motor_cfg.get("maxPowerW", 785.0))
        self.throttle_pct = float(self.motor_cfg.get("throttlePct", 100.0))
        self.lever_arm_mm = float(self.brake_cfg.get("leverArmMm", 250.0))
        self.magnet_gap_mm = float(self.brake_cfg.get("magnetGapMm", 1.0))
        self.ref_rpm = float(self.brake_cfg.get("refRPM", 1500.0))
        self.max_brake_per_magnet_nm = float(self.brake_cfg.get("maxBrakePerMagnetNm", 8.0))
        self.magnet_count = int(cfg.get("materials", {}).get("magnetCount", 2))
        self.inertia = float(self.numerics_cfg.get("inertiaKgM2", 0.0065))
        self.friction = float(self.numerics_cfg.get("frictionNm", 0.00015))
        self.bearing_friction = float(self.numerics_cfg.get("bearingFrictionNm", 0.0006))
        self.ambient_c = float(self.thermal_cfg.get("ambientC", 25.0))
        self.ir_radius_mm = float(self.measurement_cfg.get("irSensorRadiusMm", 120.0))
        self.magnet_elec_w_each = float(self.magnet_cfg.get("electricWEach", 5.2))
        self.impulses_per_rev = int(self.p2_cfg.get("industrialCounter", {}).get("impulsesPerRevolution", 20))

        self.code_text = CODE_PATH.read_text(encoding="utf-8", errors="replace") if CODE_PATH.exists() else "// main.cpp not found\n"
        self.code_version = 1
        self.debug_log: deque[str] = deque(maxlen=160)
        self.history: deque[dict[str, float]] = deque(maxlen=240)
        self.last_loop = time.time()
        self.last_history_push = self.last_loop
        self.start_time = time.time()

        self.boot_phase = "off"
        self.boot_progress = 0.0
        self.run_requested = False
        self.debug_enabled = True
        self.esp_mode = "off"
        self.manual_pwm_pct = 0.0
        self.target_rpm = 1800.0
        self.target_torque_nm = 2.0
        self.pid_kp = 0.70
        self.pid_ki = 0.20
        self.pid_kd = 0.01
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        self.omega = 0.0
        self.rpm = 0.0
        self.motor_nm = 0.0
        self.brake_nm = 0.0
        self.force_n = 0.0
        self.ir_temp_c = self.ambient_c
        self.bulk_temp_c = self.ambient_c
        self.magnet_pwm_pct = 0.0
        self.motor_electric_w = 0.0
        self.magnet_electric_w = 0.0
        self.pulse_rate_hz = 0.0
        self.uptime_s = 0.0
        self.control_error_pct = 0.0
        self.uncertainty_pct = 0.0
        self.disc1_sat_pct = 0.0
        self.disc2_sat_pct = 0.0
        self.last_reason = "ESP32 aus"
        self.bricklets = {
            "esp32": bool(self.p2_cfg.get("esp32Brick", {}).get("enabled", True)),
            "industrialCounter": bool(self.p2_cfg.get("industrialCounter", {}).get("enabled", True)),
            "temperatureIrV2": bool(self.p2_cfg.get("temperatureIrV2", {}).get("enabled", True)),
            "loadCellV2": bool(self.p2_cfg.get("loadCellV2", {}).get("enabled", True)),
            "isolator": bool(self.p2_cfg.get("isolator", {}).get("enabled", True)),
            "dcV2": bool(self.p2_cfg.get("dcV2", {}).get("enabled", True)),
            "industrialAnalogOutV2": bool(self.p2_cfg.get("industrialAnalogOutV2", {}).get("enabled", True)),
            "oled64x48": bool(self.p2_cfg.get("oled64x48", {}).get("enabled", True)),
        }
        self.oled_lines = ["TF Brake Stand", "Mode: OFF", "F 0.0 N", "n 0 RPM"]
        self._log("P2-Bridge initialisiert")

    def _log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.debug_log.appendleft(f"[{stamp}] {message}")

    def _motor_stall_nm(self) -> float:
        omega0 = max(2 * math.pi * self.max_rpm / 60.0, 1e-6)
        return max((4.0 * self.max_power_w) / omega0, 0.05)

    def _motor_moment_nm(self) -> float:
        if self.boot_phase != "run" or not self.run_requested:
            return 0.0
        throttle = clamp(self.throttle_pct / 100.0, 0.0, 1.0)
        omega0 = max(2 * math.pi * self.max_rpm / 60.0 * max(throttle, 0.05), 1e-6)
        stall = self._motor_stall_nm() * throttle
        if self.omega >= omega0:
            return 0.0
        return max(0.0, stall * (1.0 - self.omega / omega0))

    def _brake_capacity_nm(self) -> float:
        rpm_factor = max(self.rpm / max(self.ref_rpm, 1.0), 0.05)
        gap_factor = clamp((0.55 / max(self.magnet_gap_mm, 0.5)) ** 0.75, 0.65, 2.0)
        temp_factor = clamp(1.0 - max(self.bulk_temp_c - self.ambient_c, 0.0) * 0.0012, 0.65, 1.0)
        return self.max_brake_per_magnet_nm * self.magnet_count * rpm_factor * gap_factor * temp_factor

    def _brake_moment_nm(self, pwm_pct: float) -> float:
        ratio = clamp(pwm_pct / 100.0, 0.0, 1.0)
        field = math.tanh(1.6 * ratio) / math.tanh(1.6)
        return self._brake_capacity_nm() * field * field

    def _motor_electric_power_w(self) -> float:
        if not self.run_requested or self.boot_phase != "run" or self.esp_mode == "off":
            return 0.0
        shaft = self.motor_nm * self.omega
        throttle = clamp(self.throttle_pct / 100.0, 0.0, 1.0)
        idle = 1.8 + 0.018 * self.max_power_w * throttle
        eta = clamp(0.58 + 0.24 * clamp(shaft / max(self.max_power_w, 1.0), 0.0, 1.0), 0.45, 0.88)
        return 0.0 if throttle <= 0.0 and shaft <= 0.0 else idle + shaft / eta

    def _conductivity_temp_factor(self) -> float:
        coeff = float(self.thermal_cfg.get("conductivityTempCoeffPerC", 0.0038))
        return clamp(1.0 - max(self.bulk_temp_c - self.ambient_c, 0.0) * coeff, 0.55, 1.02)

    def _magnet_flux_temp_factor(self) -> float:
        coeff = float(self.thermal_cfg.get("magnetFluxTempCoeffPerC", 0.0012))
        return clamp(1.0 - max(self.bulk_temp_c - self.ambient_c, 0.0) * coeff, 0.65, 1.02)

    def _skin_depth_m(self) -> float:
        sigma = self._conductivity_temp_factor() * float(self.materials_cfg.get("aluConductivitySPerM", 3.5e7))
        if self.omega <= 1e-9:
            return 1.0
        mu0 = 4.0 * math.pi * 1e-7
        return math.sqrt(2.0 / max(mu0 * sigma * self.omega, 1e-9))

    def _disc_saturation_index(self, thickness_mm: float) -> float:
        limit_t = float(self.materials_cfg.get("workingFluxLimitT", 0.25))
        thickness_m = max(thickness_mm / 1000.0, 1e-4)
        skin_depth = max(self._skin_depth_m(), 1e-5)
        shape = float(self.materials_cfg.get("magneticFieldShape", 1.6))
        field_share = math.tanh(shape * clamp(self.magnet_pwm_pct / 100.0, 0.0, 1.0)) / math.tanh(shape)
        local_field = limit_t * field_share * self._magnet_flux_temp_factor()
        crit_speed = max(self.ref_rpm * float(self.materials_cfg.get("criticalSpeedFactorBase", 1.6)), 600.0)
        speed_term = math.sqrt(max(self.rpm / crit_speed, 0.0))
        return clamp((local_field / max(limit_t, 1e-6)) * (thickness_m / max(2.0 * skin_depth, 1e-6)) * speed_term, 0.0, 2.0)

    def _uncertainty_fraction(self) -> float:
        lever_tol_mm = float(self.measurement_cfg.get("leverArmToleranceMm", 1.0))
        load_offset = float(self.measurement_cfg.get("loadCellOffsetN", 0.0))
        mis_deg = float(self.measurement_cfg.get("loadCellMisalignmentDeg", 0.0))
        nonlin_pct = float(self.measurement_cfg.get("loadCellNonlinearityPctFS", 0.6)) / 100.0
        rated_n = max(float(self.measurement_cfg.get("loadCellRatedN", 50.0)), 1.0)
        lever_rel = abs(lever_tol_mm) / max(self.lever_arm_mm, 1.0)
        angle_rel = abs(1.0 - math.cos(math.radians(mis_deg)))
        offset_rel = abs(load_offset) / max(abs(self.force_n), rated_n * 0.1, 0.1)
        nonlin_rel = nonlin_pct * clamp(abs(self.force_n) / rated_n, 0.0, 1.2)
        temp_rel = max(abs(1.0 - self._conductivity_temp_factor()), abs(1.0 - self._magnet_flux_temp_factor()))
        return math.sqrt(lever_rel**2 + angle_rel**2 + offset_rel**2 + nonlin_rel**2 + temp_rel**2)

    def _physics_summary(self) -> tuple[str, str, str, str]:
        dominant_disc = "Scheibe 1" if self.disc1_sat_pct >= self.disc2_sat_pct else "Scheibe 2"
        max_sat = max(self.disc1_sat_pct, self.disc2_sat_pct)
        if self.uncertainty_pct >= 12.0:
            return (
                "UNSICHER",
                f"Messkette streut mit {self.uncertainty_pct:.1f} %. Hebelarm, Waegezelle oder Temperaturkorrektur dominieren.",
                "Hebelarm/Waegezelle kalibrieren, Offset nullen, Scheibe kuehlen.",
                dominant_disc,
            )
        if max_sat >= 90.0:
            return (
                "KRITISCH",
                f"{dominant_disc} ist magnetisch kritisch belastet ({max_sat:.0f} %). Eindringtiefe sinkt und Reserve nimmt ab.",
                "Gap vergroessern, Drehzahl senken oder Scheibe kuehlen.",
                dominant_disc,
            )
        if max_sat >= 65.0:
            return (
                "HOCH",
                f"{dominant_disc} arbeitet mit reduzierter elektromagnetischer Reserve ({max_sat:.0f} %).",
                "Temperatur beobachten und Scheibendicke / Gap pruefen.",
                dominant_disc,
            )
        return (
            "OK",
            "Messkette und Bremsscheiben arbeiten im plausiblen Bereich.",
            "Keine Gegenmassnahme noetig.",
            dominant_disc,
        )

    def _update_oled(self) -> None:
        self.oled_lines = [
            f"F {self.force_n:4.1f} N",
            f"n {self.rpm:4.0f} RPM",
            f"Imp {self.pulse_rate_hz:4.0f}/s",
            f"M {self.brake_nm:4.2f}Nm"
        ]

    def _push_history(self, now: float) -> None:
        if now - self.last_history_push < 0.25:
            return
        self.last_history_push = now
        self.history.append(
            {
                "time": round(self.uptime_s, 3),
                "rpm": round(self.rpm, 3),
                "forceN": round(self.force_n, 4),
                "pwmPct": round(self.magnet_pwm_pct, 3),
                "torqueNm": round(self.brake_nm, 4),
                "irTempC": round(self.ir_temp_c, 3),
            }
        )

    def _boot_step(self, dt: float) -> None:
        if not self.run_requested:
            self.boot_phase = "off"
            self.boot_progress = 0.0
            self.last_reason = "ESP32 aus"
            return

        phases = ["power", "bootloader", "init-bricklets", "run"]
        if self.boot_phase == "off":
            self.boot_phase = "power"
            self.boot_progress = 0.0
            self._log("24V Versorgung an, ESP32 Brick bootet")

        self.boot_progress += dt * 0.55
        phase_index = phases.index(self.boot_phase)
        if self.boot_progress >= 1.0 and phase_index < len(phases) - 1:
            self.boot_progress = 0.0
            self.boot_phase = phases[phase_index + 1]
            if self.boot_phase == "bootloader":
                self._log("Bootloader aktiv, Firmware wird geladen")
            elif self.boot_phase == "init-bricklets":
                self._log("Bricklet-Ports werden enumeriert")
            elif self.boot_phase == "run":
                self._log("Firmware im Run-Modus")

        if self.boot_phase != "run":
            self.last_reason = {
                "power": "Versorgung und Brick initialisieren",
                "bootloader": "Firmware laden",
                "init-bricklets": "Bricklets werden erkannt"
            }.get(self.boot_phase, "Startet")

    def _controller_step(self, dt: float) -> None:
        if self.boot_phase != "run" or self.esp_mode == "off":
            self.magnet_pwm_pct = 0.0
            self.control_error_pct = 0.0
            self.pid_integral = 0.0
            self.pid_last_error = 0.0
            return

        if self.esp_mode == "manual":
            self.magnet_pwm_pct = clamp(self.manual_pwm_pct, 0.0, 100.0)
            self.last_reason = f"Manuell: PWM {self.magnet_pwm_pct:.0f}%"
            return

        if self.esp_mode == "rpm":
            error = self.rpm - self.target_rpm
            self.control_error_pct = 100.0 * error / max(self.target_rpm, 1.0)
            if error < 0:
                self.magnet_pwm_pct = 0.0
                self.pid_integral = 0.0
                self.pid_last_error = 0.0
                self.last_reason = "Ziel-RPM unter Last nicht bremsend beeinflusst"
                return
        else:
            error = self.target_torque_nm - self.brake_nm
            self.control_error_pct = 100.0 * error / max(self.target_torque_nm, 0.05)

        self.pid_integral = clamp(self.pid_integral + error * dt, -200.0, 200.0)
        deriv = (error - self.pid_last_error) / max(dt, 1e-6)
        output = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * deriv
        self.pid_last_error = error
        self.magnet_pwm_pct = clamp(self.magnet_pwm_pct + output, 0.0, 100.0)
        if self.esp_mode == "rpm":
            self.last_reason = f"Regelt auf Drehzahl {self.target_rpm:.0f} RPM"
        else:
            self.last_reason = f"Regelt auf Drehmoment {self.target_torque_nm:.2f} Nm"

    def _loop(self) -> None:
        while self._running:
            now = time.time()
            dt = min(now - self.last_loop, 0.05)
            self.last_loop = now
            with self._lock:
                self.uptime_s = now - self.start_time
                self._boot_step(dt)
                self._controller_step(dt)
                self.motor_nm = self._motor_moment_nm()
                self.brake_nm = self._brake_moment_nm(self.magnet_pwm_pct)
                friction = self.friction + self.bearing_friction + self.omega * 0.00002
                net = self.motor_nm - self.brake_nm - friction
                self.omega = max(0.0, self.omega + (net / max(self.inertia, 1e-6)) * dt)
                self.rpm = self.omega * 60.0 / (2.0 * math.pi)
                self.force_n = self.brake_nm / max(self.lever_arm_mm / 1000.0, 0.05)
                self.pulse_rate_hz = self.rpm / 60.0 * self.impulses_per_rev
                self.magnet_electric_w = self.magnet_count * self.magnet_elec_w_each * clamp(self.magnet_pwm_pct / 100.0, 0.0, 1.0)
                self.motor_electric_w = self._motor_electric_power_w()
                p_diss = self.brake_nm * self.omega
                self.bulk_temp_c += ((p_diss / 300.0) - (self.bulk_temp_c - self.ambient_c) * 0.08) * dt
                self.bulk_temp_c = max(self.ambient_c, self.bulk_temp_c)
                self.ir_temp_c += ((self.bulk_temp_c + p_diss / 1000.0 * 14.0) - self.ir_temp_c) * dt * 0.8
                self.uncertainty_pct = self._uncertainty_fraction() * 100.0
                self.disc1_sat_pct = self._disc_saturation_index(float(self.brake_cfg.get("disc1ThicknessMm", 2.0))) * 100.0
                self.disc2_sat_pct = self._disc_saturation_index(float(self.brake_cfg.get("disc2ThicknessMm", 2.0))) * 100.0
                self._update_oled()
                self._push_history(now)
            time.sleep(0.02)

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            physics_state, physics_explain, physics_action, dominant_disc = self._physics_summary()
            disc1_explain = (
                f"Scheibe 1 bei {self.disc1_sat_pct:.0f} %. "
                f"Einfluss: RPM {self.rpm:.0f}, Gap {self.magnet_gap_mm:.2f} mm, "
                f"d1 {float(self.brake_cfg.get('disc1ThicknessMm', 2.0)):.1f} mm, IR {self.ir_temp_c:.1f} C."
            )
            disc2_explain = (
                f"Scheibe 2 bei {self.disc2_sat_pct:.0f} %. "
                f"Einfluss: RPM {self.rpm:.0f}, Gap {self.magnet_gap_mm:.2f} mm, "
                f"d2 {float(self.brake_cfg.get('disc2ThicknessMm', 2.0)):.1f} mm, IR {self.ir_temp_c:.1f} C."
            )
            uncertainty_explain = (
                f"Gesamt {self.uncertainty_pct:.1f} %. "
                "Dominant: Hebelarm/Waagezelle/Temperaturmodell."
            )
            if self.uncertainty_pct >= 12.0:
                uncertainty_explain += " Empfehlung: Waage nullen, Hebelarm nachmessen, Scheibe kuehlen."
            elif self.uncertainty_pct >= 5.0:
                uncertainty_explain += " Empfehlung: Kalibrierpunkte aufnehmen und Temperatur beobachten."
            return {
                "online": True,
                "bootPhase": self.boot_phase,
                "bootProgress": round(self.boot_progress, 3),
                "runRequested": self.run_requested,
                "espMode": self.esp_mode,
                "manualPwmPct": round(self.manual_pwm_pct, 2),
                "magnetPwmPct": round(self.magnet_pwm_pct, 2),
                "targetRpm": round(self.target_rpm, 1),
                "targetTorqueNm": round(self.target_torque_nm, 3),
                "rpm": round(self.rpm, 3),
                "omega": round(self.omega, 4),
                "motorNm": round(self.motor_nm, 4),
                "brakeNm": round(self.brake_nm, 4),
                "forceN": round(self.force_n, 4),
                "irTempC": round(self.ir_temp_c, 3),
                "bulkTempC": round(self.bulk_temp_c, 3),
                "pulseRateHz": round(self.pulse_rate_hz, 3),
                "impulsesPerRev": self.impulses_per_rev,
                "motorElectricW": round(self.motor_electric_w, 3),
                "magnetElectricW": round(self.magnet_electric_w, 3),
                "uptimeS": round(self.uptime_s, 2),
                "controlErrorPct": round(self.control_error_pct, 3),
                "uncertaintyPct": round(self.uncertainty_pct, 3),
                "disc1SatPct": round(self.disc1_sat_pct, 3),
                "disc2SatPct": round(self.disc2_sat_pct, 3),
                "uncertaintyExplain": uncertainty_explain,
                "disc1Explain": disc1_explain,
                "disc2Explain": disc2_explain,
                "physicsCheck": f"{physics_state}: {physics_explain} {physics_action}",
                "controlExplain": (
                    f"Modus {self.esp_mode} | Soll "
                    f"{self.target_rpm:.0f} RPM / {self.target_torque_nm:.2f} Nm | "
                    f"Abweichung {self.control_error_pct:.1f} % | dominant {dominant_disc}"
                ),
                "lastReason": self.last_reason,
                "bricklets": self.bricklets,
                "oledLines": list(self.oled_lines),
                "debugLines": list(self.debug_log)[:8],
                "codeVersion": self.code_version,
                "trend": list(self.history)[-36:],
            }

    def get_code(self) -> dict[str, Any]:
        with self._lock:
            return {
                "codeText": self.code_text,
                "codeVersion": self.code_version,
            }

    def handle_action(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = payload.get("action")
        with self._lock:
            if action == "run":
                self.run_requested = True
                self._log("RUN angefordert")
            elif action == "stop":
                self.run_requested = False
                self.esp_mode = "off"
                self.magnet_pwm_pct = 0.0
                self.manual_pwm_pct = 0.0
                self._log("STOP angefordert")
            elif action == "reset":
                self._load_config()
            elif action == "set_mode":
                mode = payload.get("mode", "off")
                if mode in {"off", "manual", "rpm", "torque"}:
                    self.esp_mode = mode
                    self._log(f"ESP-Modus -> {mode}")
            elif action == "set_manual_pwm":
                self.manual_pwm_pct = clamp(float(payload.get("value", 0.0)), 0.0, 100.0)
                self._log(f"Manuelle PWM -> {self.manual_pwm_pct:.0f}%")
            elif action == "set_target_rpm":
                self.target_rpm = clamp(float(payload.get("value", self.target_rpm)), 0.0, self.max_rpm)
                self._log(f"Ziel-RPM -> {self.target_rpm:.0f}")
            elif action == "set_target_torque":
                self.target_torque_nm = clamp(float(payload.get("value", self.target_torque_nm)), 0.0, 100.0)
                self._log(f"Ziel-Drehmoment -> {self.target_torque_nm:.2f} Nm")
            elif action == "set_code":
                self.code_text = str(payload.get("code", self.code_text))
                self.code_version += 1
                self._log("Codefenster aktualisiert")
            elif action == "debug_step":
                self._log("Debug-Step simuliert")
            return self.get_state()


SIM = P2Simulator()


class Handler(BaseHTTPRequestHandler):
    def _send(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        if self.path.startswith("/api/p2/state"):
            self._send(200, SIM.get_state())
            return
        if self.path.startswith("/api/p2/code"):
            self._send(200, SIM.get_code())
            return
        self._send(404, {"error": "not_found"})

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(body.decode("utf-8"))
        except json.JSONDecodeError:
            payload = {}
        if self.path.startswith("/api/p2/control"):
            self._send(200, SIM.handle_action(payload))
            return
        self._send(404, {"error": "not_found"})

    def log_message(self, format: str, *args: Any) -> None:
        return


def main() -> None:
    server = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
    print("[P2] Bridge server listening on http://127.0.0.1:8765")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
