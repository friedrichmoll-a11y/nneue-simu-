#!/usr/bin/env python3
"""
TinkerForge Wirbelstrombremse Simulation
=========================================
Bremsprüfstand – Virtueller TinkerForge-Stack (kein Hardware nötig)

Physikmodell Wirbelstrombremse
-------------------------------
Bremsmoment:      M_brake = k_eddy * omega * (PWM/255)^2
                  [Wirbelströme ~ omega, Kraft ~ B^2 ~ I^2 ~ U^2]
Scheibenrotation: J * d(omega)/dt = M_motor - M_brake - M_reib
Temperatur:       dT/dt = P_diss/(m*c_p) - (T - T_amb)/R_th
                  P_diss = M_brake * omega  [Bremsleistung in Wärme]

Simulierte TinkerForge-Bricklets (Mock-API, echte TF-Namen/Konventionen)
--------------------------------------------------------------------------
  TFIndustrialDualAnalogIn20  (UID: TFAI1)  → SPS-Sollwerte 0–10 V
  TFIndustrialAnalogOut20     (UID: TFAO1)  → Istwert-Ausgabe 0–10 V
  TFTemperatureBricklet20     (UID: TFTMP1) → Scheibentemperatur
  TFIndustrialCounterBricklet (UID: TFCNT1) → Drehzahlpulse

Kommandos (Serial-kompatibel mit sketch.ino)
---------------------------------------------
  b0-255   Magnet-PWM direkt setzen
  r        Magnet reset (PWM=0)
  t        Tara Wägezelle
  l200-400 Hebelarm Wägezelle [mm]
  s0/1     Simulation OFF/ON
  m0-10    S7 SollMoment [Nm]
  n0-6000  S7 SollDrehzahl [RPM]
  on/off   S7 Freigabe
  x        S7 Reset
  tf       TinkerForge Gerätestatus
  h        Hilfe

Abhängigkeiten: Nur Python-Stdlib (threading, time, math, sys)
"""

import time
import sys
import math
import threading
from datetime import datetime


# ─────────────────────────────────────────────
#  Physik-Konstanten (reale Scheibe ~Ø300mm)
# ─────────────────────────────────────────────
class PhysikParameter:
    # Scheibe
    SCHEIBEN_RADIUS_M      = 0.150      # m
    SCHEIBEN_DICKE_M       = 0.010      # m
    SCHEIBEN_DICHTE        = 7800.0     # kg/m³ (Stahl)
    SCHEIBEN_WAERMEKAPAZ   = 500.0      # J/(kg·K)

    # Massenträgheitsmoment J = 0.5 * m * r²
    @staticmethod
    def scheiben_masse():
        m = (math.pi * PhysikParameter.SCHEIBEN_RADIUS_M**2
             * PhysikParameter.SCHEIBEN_DICKE_M
             * PhysikParameter.SCHEIBEN_DICHTE)
        return m  # ≈ 5.5 kg

    @staticmethod
    def massentraegheit_J():
        m = PhysikParameter.scheiben_masse()
        return 0.5 * m * PhysikParameter.SCHEIBEN_RADIUS_M**2  # ≈ 0.062 kg·m²

    # Wirbelstrombremse
    # k_eddy so gewählt: bei 3000 RPM + vollem PWM ≈ 50 Nm Bremsmoment
    #   omega_3000 = 3000/60 * 2*pi ≈ 314 rad/s
    #   k_eddy = 50 / (314 * 1.0) = 0.159  (PWM-Faktor = 1 bei PWM=255)
    K_EDDY                 = 0.159      # Nm·s/rad

    # Reibmoment (lagerreibung)
    M_REIBUNG_NM           = 0.05       # Nm

    # Antriebsmotor (Simulation)
    MOTOR_MAX_MOMENT_NM    = 80.0       # Nm bei Stillstand
    MOTOR_LEERLAUF_RPM     = 3000.0     # RPM bei Leerlauf

    # Thermik
    R_THERMISCH_K_W        = 0.8        # K/W (natürliche Konvektion)
    T_UMGEBUNG_C           = 25.0       # °C

    # Hebelarm (Mechanik)
    HEBELARM_MAGNET_MM     = 100.0      # mm
    HEBELARM_WAEGE_MM_DEF  = 250.0      # mm (einstellbar)

    # Temperaturgrenzen
    TEMP_WARN_C            = 80.0
    TEMP_CRIT_C            = 120.0


# ─────────────────────────────────────────────
#  TinkerForge Mock-API
#  (Gleiche Konventionen wie echte TF-Bricklets)
# ─────────────────────────────────────────────
class TFIPConnection:
    """Simulierter TinkerForge Brick Daemon (brickd)"""

    HOST    = "localhost"
    PORT    = 4223          # Standard TF-Port
    STATE_DISCONNECTED = 0
    STATE_CONNECTED    = 1

    def __init__(self):
        self._devices   = {}
        self._state     = self.STATE_CONNECTED  # Direkt verbunden (simuliert)
        self._callbacks = {}
        print(f"[TF] IPConnection simuliert → {self.HOST}:{self.PORT} (mock)")

    def connect(self, host, port):
        self._state = self.STATE_CONNECTED
        print(f"[TF] Verbunden mit {host}:{port} (simuliert)")

    def disconnect(self):
        self._state = self.STATE_DISCONNECTED

    def get_connection_state(self):
        return self._state

    def _register_device(self, uid, device):
        self._devices[uid] = device
        print(f"[TF] Gerät registriert: {device.__class__.__name__} UID={uid}")

    def enumerate(self):
        print(f"[TF] Enumerate: {len(self._devices)} Geräte gefunden")
        for uid, dev in self._devices.items():
            print(f"     {dev.__class__.__name__:35s} UID={uid}")


class TFBricklet:
    """Basis-Mock für alle TinkerForge Bricklets"""
    DEVICE_IDENTIFIER = 0

    def __init__(self, uid, ipcon):
        self._uid       = uid
        self._ipcon     = ipcon
        self._callbacks = {}
        ipcon._register_device(uid, self)

    def get_identity(self):
        return {
            "uid":                self._uid,
            "connected_uid":      "0",
            "position":           "A",
            "hardware_version":   [2, 0, 0],
            "firmware_version":   [2, 0, 0],
            "device_identifier":  self.DEVICE_IDENTIFIER,
        }

    def register_callback(self, callback_id, function):
        self._callbacks[callback_id] = function

    def _fire_callback(self, callback_id, *args):
        if callback_id in self._callbacks:
            self._callbacks[callback_id](*args)


class TFIndustrialDualAnalogIn20(TFBricklet):
    """
    Virtuelles Industrial Dual Analog In Bricklet 2.0
    Liest SPS-Ausgänge: SollMoment (CH0) und SollDrehzahl (CH1)
    Spannung 0–10 V → Werte in mV (TF-Konvention)
    """
    DEVICE_IDENTIFIER   = 2121
    CALLBACK_VOLTAGE    = 22

    def __init__(self, uid, ipcon):
        super().__init__(uid, ipcon)
        self._voltage_mv     = [0, 0]   # [CH0, CH1] in mV
        self._callback_ms    = [0, 0]
        self._last_cb_time   = [0.0, 0.0]

    def set_voltage_mv(self, channel, millivolt):
        """Intern: SPS-Sollwert setzen (0–10000 mV)"""
        self._voltage_mv[channel] = max(0, min(10000, int(millivolt)))

    def get_voltage(self, channel):
        """Spannung in mV zurückgeben (0–10000)"""
        return self._voltage_mv[channel]

    def set_voltage_callback_configuration(self, channel, period_ms, value_has_to_change, option, min_val, max_val):
        self._callback_ms[channel] = period_ms

    def tick(self, now):
        for ch in range(2):
            if (self._callback_ms[ch] > 0
                    and now - self._last_cb_time[ch] >= self._callback_ms[ch] / 1000.0):
                self._last_cb_time[ch] = now
                self._fire_callback(self.CALLBACK_VOLTAGE, ch, self._voltage_mv[ch])


class TFIndustrialAnalogOut20(TFBricklet):
    """
    Virtuelles Industrial Analog Out Bricklet 2.0
    Gibt Istwerte aus: IstMoment (0–10 V) und IstDrehzahl (0–10 V)
    """
    DEVICE_IDENTIFIER = 2116

    def __init__(self, uid, ipcon):
        super().__init__(uid, ipcon)
        self._voltage_mv = 0    # aktueller Ausgangswert in mV
        self._enabled    = True

    def set_voltage(self, millivolt):
        """Ausgangsspannung setzen (0–10000 mV)"""
        self._voltage_mv = max(0, min(10000, int(millivolt)))

    def get_voltage(self):
        return self._voltage_mv

    def set_enabled(self, enabled):
        self._enabled = enabled

    def get_enabled(self):
        return self._enabled


class TFTemperatureBricklet20(TFBricklet):
    """
    Virtuelles Temperature Bricklet 2.0
    Scheibentemperatur → Wert in 1/100 °C (TF-Konvention)
    Beispiel: 2534 = 25.34 °C
    """
    DEVICE_IDENTIFIER  = 2110
    CALLBACK_TEMPERATURE = 8

    def __init__(self, uid, ipcon):
        super().__init__(uid, ipcon)
        self._temp_100  = 2500   # in 1/100 °C → 25.00 °C
        self._cb_period = 0
        self._last_cb   = 0.0

    def set_temperature_100(self, temp_celsius):
        """Intern: Temperatur in °C setzen"""
        self._temp_100 = int(temp_celsius * 100)

    def get_temperature(self):
        """Temperatur in 1/100 °C"""
        return self._temp_100

    def get_temperature_celsius(self):
        """Komfort-Methode: Temperatur in °C"""
        return self._temp_100 / 100.0

    def set_temperature_callback_configuration(self, period_ms, value_has_to_change, option, min_val, max_val):
        self._cb_period = period_ms

    def tick(self, now):
        if (self._cb_period > 0
                and now - self._last_cb >= self._cb_period / 1000.0):
            self._last_cb = now
            self._fire_callback(self.CALLBACK_TEMPERATURE, self._temp_100)


class TFIndustrialCounterBricklet(TFBricklet):
    """
    Virtuelles Industrial Counter Bricklet
    Zählt Drehzahlpulse (Rising Edges)
    Drehzahlberechnung: n [RPM] = count_delta / dt_s * 60 / pulses_per_rev
    """
    DEVICE_IDENTIFIER    = 293
    CALLBACK_ALL_COUNTER = 10

    def __init__(self, uid, ipcon, pulses_per_rev=1):
        super().__init__(uid, ipcon)
        self._count          = [0, 0, 0, 0]   # 4 Kanäle
        self._pulses_per_rev = pulses_per_rev
        self._cb_period      = 0
        self._last_cb        = 0.0

    def add_pulses(self, channel, count):
        """Intern: Pulse addieren"""
        self._count[channel] += int(count)

    def get_all_counter(self, reset=False):
        result = list(self._count)
        if reset:
            self._count = [0, 0, 0, 0]
        return result

    def get_counter(self, channel, reset=False):
        val = self._count[channel]
        if reset:
            self._count[channel] = 0
        return val

    def set_all_counter_callback_configuration(self, period_ms):
        self._cb_period = period_ms

    def tick(self, now):
        if (self._cb_period > 0
                and now - self._last_cb >= self._cb_period / 1000.0):
            self._last_cb = now
            self._fire_callback(self.CALLBACK_ALL_COUNTER, *self._count)


# ─────────────────────────────────────────────
#  Physik-Engine: Wirbelstrombremse
# ─────────────────────────────────────────────
class WirbelstromBremsPhysik:
    """
    Physikalisches Modell einer Wirbelstrombremse

    Merksatz: „Mehr Drehzahl → mehr Wirbelstrom → mehr Bremsung"
    Das Bremsmoment wächst linear mit omega (daher: kein fester Widerstand,
    sondern ein geschwindigkeitsabhängiger Dämpfer).

    Formeln:
      PWM-Faktor:  f = PWM/255            (0…1)
      Bremsmoment: M_b = k_eddy * f² * ω  [Nm]
      Motor:       M_m = M_max*(1 - ω/ω_max)
      Dynamik:     dω/dt = (M_m - M_b - M_reib) / J
      Temperatur:  dT/dt = (M_b*ω)/(m*c_p) - (T-T_amb)/R_th
    """

    def __init__(self):
        p = PhysikParameter
        self.J          = p.massentraegheit_J()          # ≈ 0.062 kg·m²
        self.m_scheibe  = p.scheiben_masse()             # ≈ 5.5 kg
        self.c_p        = p.SCHEIBEN_WAERMEKAPAZ         # 500 J/(kg·K)
        self.R_th       = p.R_THERMISCH_K_W              # 0.8 K/W
        self.T_amb      = p.T_UMGEBUNG_C
        self.k_eddy     = p.K_EDDY
        self.M_reib     = p.M_REIBUNG_NM
        self.M_max      = p.MOTOR_MAX_MOMENT_NM
        self.omega_max  = p.MOTOR_LEERLAUF_RPM / 60.0 * 2 * math.pi  # rad/s

        # Zustand
        self.omega      = 0.0      # rad/s
        self.temp_C     = p.T_UMGEBUNG_C
        self.pwm        = 0        # 0…255

    def rpm(self):
        return self.omega / (2 * math.pi) * 60.0

    def bremsmoment_Nm(self):
        """M_b = k_eddy * (PWM/255)² * ω"""
        f = self.pwm / 255.0
        return self.k_eddy * f * f * self.omega

    def motormoment_Nm(self):
        """Lineares Motorkennlinienmodell"""
        if self.omega >= self.omega_max:
            return 0.0
        return self.M_max * (1.0 - self.omega / self.omega_max)

    def leistung_W(self):
        """Bremsleistung (wird in Wärme umgewandelt)"""
        return self.bremsmoment_Nm() * self.omega

    def schritt(self, dt_s):
        """
        Zeitschritt dt_s [Sekunden] integrieren (Euler-Vorwärts)

        Drehzahl-Update:
          dω = (M_m - M_b - M_reib) / J * dt

        Temperatur-Update:
          dT = P_diss/(m*c_p)*dt - (T-T_amb)/R_th * dt/...
               genauer: dT = [P_diss - (T-T_amb)/R_th] / (m*c_p) * dt
        """
        M_b     = self.bremsmoment_Nm()
        M_m     = self.motormoment_Nm()
        M_netto = M_m - M_b - self.M_reib

        # Drehzahl (omega kann nicht negativ werden)
        self.omega += (M_netto / self.J) * dt_s
        if self.omega < 0.0:
            self.omega = 0.0
        if self.omega > self.omega_max * 2.5:
            self.omega = self.omega_max * 2.5

        # Temperatur
        P_diss  = M_b * self.omega
        P_kuhl  = (self.temp_C - self.T_amb) / self.R_th
        dT      = (P_diss - P_kuhl) / (self.m_scheibe * self.c_p) * dt_s
        self.temp_C += dT
        if self.temp_C < self.T_amb:
            self.temp_C = self.T_amb


# ─────────────────────────────────────────────
#  Haupt-Simulator
# ─────────────────────────────────────────────
class TFBremspruefstandSim:
    """
    Bremsprüfstand mit virtuellem TinkerForge-Stack

    Verbindet Physik-Engine mit TF-Mock-Bricklets und
    repliziert das Serial-Protokoll aus sketch.ino.
    """

    def __init__(self):
        # TF-Stack aufbauen
        self.ipcon = TFIPConnection()
        self.ai    = TFIndustrialDualAnalogIn20("TFAI1",  self.ipcon)   # SPS-Eingänge
        self.ao    = TFIndustrialAnalogOut20("TFAO1",     self.ipcon)   # SPS-Ausgänge
        self.temp  = TFTemperatureBricklet20("TFTMP1",    self.ipcon)   # Temperatur
        self.cnt   = TFIndustrialCounterBricklet("TFCNT1", self.ipcon)  # Drehzahl

        # TF-Callbacks konfigurieren
        self.temp.set_temperature_callback_configuration(500, False, "x", 0, 0)
        self.cnt.set_all_counter_callback_configuration(500)
        self.ai.set_voltage_callback_configuration(0, 100, False, "x", 0, 10000)
        self.ai.set_voltage_callback_configuration(1, 100, False, "x", 0, 10000)

        # Physik
        self.physik = WirbelstromBremsPhysik()

        # Mechanik
        self.hebelarm_waege_mm  = PhysikParameter.HEBELARM_WAEGE_MM_DEF
        self.hebelarm_magnet_mm = PhysikParameter.HEBELARM_MAGNET_MM

        # S7-Signale
        self.S7_SollMoment_Nm     = 0.0
        self.S7_SollDrehzahl_RPM  = 0.0
        self.S7_Freigabe          = False

        # Ausgabe-Cache
        self._last_rpm_for_cnt    = 0.0
        self._pulse_accumulator   = 0.0

        # Timing
        self._start_time  = time.time()
        self._last_sim    = time.time()
        self._last_print  = 0.0
        self._last_tick   = 0.0
        self._lauf        = True

        print("\n=== TinkerForge Wirbelstrombremse Simulation ===")
        print(f"Start: {datetime.now().strftime('%H:%M:%S')}")
        print(f"Scheibe: Ø{PhysikParameter.SCHEIBEN_RADIUS_M*2*1000:.0f}mm, "
              f"m={self.physik.m_scheibe:.2f}kg, J={self.physik.J*1000:.1f}g·m²")
        print(f"k_eddy={self.physik.k_eddy:.4f} Nm·s/rad  "
              f"(M_max@3000RPM ≈ {self.physik.k_eddy*1.0*(3000/60*2*math.pi):.1f} Nm)")
        print(f"TF-Bricklets: AI={self.ai._uid} AO={self.ao._uid} "
              f"Temp={self.temp._uid} Cnt={self.cnt._uid}\n")
        self._print_help()

    # ── Hilfsmethoden ────────────────────────────────────
    def _mv_to_nm(self, millivolt, max_nm=10.0):
        """0–10000 mV → 0–max_nm"""
        return millivolt / 10000.0 * max_nm

    def _nm_to_mv(self, nm, max_nm=10.0):
        """0–max_nm Nm → 0–10000 mV"""
        return int(min(10000, max(0, nm / max_nm * 10000.0)))

    def _rpm_to_mv(self, rpm, max_rpm=6000.0):
        """0–6000 RPM → 0–10000 mV"""
        return int(min(10000, max(0, rpm / max_rpm * 10000.0)))

    # ── Simulations-Loop ─────────────────────────────────
    def _sim_loop(self):
        while self._lauf:
            now    = time.time()
            dt_s   = now - self._last_sim
            self._last_sim = now

            # ── S7-Freigabe → PWM ──
            if self.S7_Freigabe:
                pwm_s7 = int((self.S7_SollMoment_Nm / 10.0) * 255)
                self.physik.pwm = max(0, min(255, pwm_s7))

            # ── Physik-Schritt ──
            self.physik.schritt(dt_s)

            # ── Pulse für Counter akkumulieren ──
            rpm_aktuell = self.physik.rpm()
            pulses_dt   = rpm_aktuell / 60.0 * dt_s   # Umdrehungen in dt
            self._pulse_accumulator += pulses_dt
            if self._pulse_accumulator >= 1.0:
                ganzzahl = int(self._pulse_accumulator)
                self.cnt.add_pulses(0, ganzzahl)
                self._pulse_accumulator -= ganzzahl

            # ── TF-Bricklets aktualisieren ──
            moment_Nm   = self.physik.bremsmoment_Nm()
            r_waage_m   = self.hebelarm_waege_mm / 1000.0
            kraft_N     = moment_Nm / r_waage_m if r_waage_m > 0 else 0.0

            self.temp.set_temperature_100(self.physik.temp_C)
            self.ao.set_voltage(self._nm_to_mv(moment_Nm))

            # SPS-Sollwerte → AI-Bricklet (0–10 V)
            self.ai.set_voltage_mv(0, self._nm_to_mv(self.S7_SollMoment_Nm))
            self.ai.set_voltage_mv(1, self._rpm_to_mv(self.S7_SollDrehzahl_RPM))

            # TF-Callbacks feuern
            self.temp.tick(now)
            self.cnt.tick(now)
            self.ai.tick(now)

            # ── Serielle Ausgabe (1 Hz, sketch.ino-kompatibel) ──
            if now - self._last_print >= 1.0:
                self._last_print = now
                self._print_status()

            time.sleep(0.005)  # 5ms → 200 Hz Physik-Takt

    # ── Ausgabe ──────────────────────────────────────────
    def _print_status(self):
        rpm     = self.physik.rpm()
        M       = self.physik.bremsmoment_Nm()
        T       = self.physik.temp_C
        pwm     = self.physik.pwm
        P       = self.physik.leistung_W()

        temp_flag = ""
        if T >= PhysikParameter.TEMP_CRIT_C:
            temp_flag = " ⚠ CRITICAL"
        elif T >= PhysikParameter.TEMP_WARN_C:
            temp_flag = " ⚠ WARN"

        # Status-Byte (wie sketch.ino)
        status = 0x00
        if T >= PhysikParameter.TEMP_WARN_C: status |= 0x02
        if T >= PhysikParameter.TEMP_CRIT_C: status |= 0x04

        # TF-Werte
        temp_100  = self.temp.get_temperature()       # 1/100 °C
        cnt_val   = self.cnt.get_counter(0)           # Gesamt-Pulse seit Start
        ao_mv     = self.ao.get_voltage()             # Istwert-Ausgang mV
        ai0_mv    = self.ai.get_voltage(0)            # SollMoment mV
        ai1_mv    = self.ai.get_voltage(1)            # SollDrehzahl mV

        print(f"RPM:{rpm:5.0f}  M:{M:5.2f}Nm  T:{T:5.1f}°C{temp_flag}"
              f"  PWM:{pwm:3d}  P:{P:6.1f}W  l:{self.hebelarm_waege_mm:.0f}mm")
        print(f"  TF: temp={temp_100/100:.2f}°C  cnt={cnt_val:6d}  "
              f"ao={ao_mv:5d}mV  ai=[{ai0_mv:5d},{ai1_mv:5d}]mV")
        print(f"  S7: Soll M={self.S7_SollMoment_Nm:.1f}Nm "
              f"n={self.S7_SollDrehzahl_RPM:.0f}RPM  "
              f"Ist M={M:.2f}Nm n={rpm:.0f}RPM  "
              f"Status=0x{status:02X}")

    def _print_help(self):
        print("─" * 50)
        print(" b0-255   Magnet PWM direkt")
        print(" r        Reset Magnet (PWM=0)")
        print(" t        Tara Wägezelle")
        print(" l200-400 Hebelarm [mm]")
        print(" m0-10    S7 SollMoment [Nm]")
        print(" n0-6000  S7 SollDrehzahl [RPM]")
        print(" on/off   S7 Freigabe")
        print(" x        S7 Reset")
        print(" tf       TF Gerätestatus")
        print(" q        Beenden")
        print("─" * 50)

    def _print_tf_status(self):
        print("\n[TF] Gerätestatus:")
        print(f"  IPConnection: {'VERBUNDEN' if self.ipcon.get_connection_state() else 'GETRENNT'}")
        self.ipcon.enumerate()
        print(f"  Temperatur:   {self.temp.get_temperature_celsius():.2f} °C  "
              f"(raw: {self.temp.get_temperature()} × 1/100 °C)")
        print(f"  Counter[0]:   {self.cnt.get_counter(0)} Pulse")
        print(f"  AnalogOut:    {self.ao.get_voltage()} mV → "
              f"{self.ao.get_voltage()/1000:.2f} V")
        print(f"  AnalogIn CH0: {self.ai.get_voltage(0)} mV (SollMoment)")
        print(f"  AnalogIn CH1: {self.ai.get_voltage(1)} mV (SollDrehzahl)\n")

    # ── Kommando-Verarbeitung ─────────────────────────────
    def process_command(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return

        if cmd.startswith("b"):
            try:
                self.physik.pwm = max(0, min(255, int(cmd[1:])))
                print(f"  → Magnet PWM = {self.physik.pwm}")
            except ValueError:
                print("  Fehler: Ungültiger PWM-Wert")

        elif cmd == "r":
            self.physik.pwm = 0
            print("  → Reset OK (PWM=0)")

        elif cmd == "t":
            print("  → Tara OK (Wägezellen-Nullpunkt gesetzt)")

        elif cmd.startswith("l"):
            try:
                val = float(cmd[1:])
                if 200.0 <= val <= 400.0:
                    self.hebelarm_waege_mm = val
                    print(f"  → Hebelarm Wägezelle = {val:.1f} mm")
                else:
                    print("  Fehler: Hebelarm muss 200–400 mm sein")
            except ValueError:
                print("  Fehler: Ungültiger Hebelarm-Wert")

        elif cmd.startswith("m"):
            try:
                val = max(0.0, min(10.0, float(cmd[1:])))
                self.S7_SollMoment_Nm = val
                print(f"  → S7 SollMoment = {val:.2f} Nm "
                      f"→ {self._nm_to_mv(val)} mV (AI CH0)")
            except ValueError:
                print("  Fehler: Ungültiger Moment-Wert")

        elif cmd.startswith("n"):
            try:
                val = max(0.0, min(6000.0, float(cmd[1:])))
                self.S7_SollDrehzahl_RPM = val
                print(f"  → S7 SollDrehzahl = {val:.0f} RPM "
                      f"→ {self._rpm_to_mv(val)} mV (AI CH1)")
            except ValueError:
                print("  Fehler: Ungültiger Drehzahl-Wert")

        elif cmd == "on":
            self.S7_Freigabe = True
            print("  → S7 Freigabe EIN (PWM folgt SollMoment)")

        elif cmd == "off":
            self.S7_Freigabe = False
            self.physik.pwm  = 0
            print("  → S7 Freigabe AUS (PWM=0)")

        elif cmd == "x":
            print("  → S7 Reset-Impuls")

        elif cmd == "tf":
            self._print_tf_status()

        elif cmd in ("h", "help", "?"):
            self._print_help()

        else:
            print(f"  Unbekanntes Kommando: '{cmd}'")
            self._print_help()

    # ── Start ──────────────────────────────────────────────
    def start(self):
        sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        sim_thread.start()

        try:
            while True:
                cmd = input("> ").strip()
                if cmd.lower() == "q":
                    break
                self.process_command(cmd)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            self._lauf = False
            print("\n=== TF Simulation beendet ===")


# ─────────────────────────────────────────────
#  Schnell-Demo: Zeige Physik ohne Eingabe
# ─────────────────────────────────────────────
def demo_mode():
    """
    Demo: Rampe von 0 → 200 PWM → 0, Ausgabe alle 2s
    Nützlich um die Physik ohne interaktive Eingabe zu testen.
    """
    print("\n[DEMO] Automatische PWM-Rampe (Ctrl+C zum Stopp)")
    p    = WirbelstromBremsPhysik()
    dt   = 0.005  # 5ms
    pwm  = 0
    t    = 0.0
    t_last_print = 0.0

    try:
        while True:
            # Rampe 0→200→0 in 60s
            t_period = t % 60.0
            pwm = int(200 * math.sin(math.pi * t_period / 60.0) ** 2)
            p.pwm = pwm
            p.schritt(dt)
            t += dt

            if t - t_last_print >= 2.0:
                t_last_print = t
                print(f"t={t:5.1f}s  PWM={pwm:3d}  "
                      f"RPM={p.rpm():5.0f}  "
                      f"M={p.bremsmoment_Nm():5.2f}Nm  "
                      f"P={p.leistung_W():6.1f}W  "
                      f"T={p.temp_C:5.1f}°C")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nDemo beendet.")


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--demo":
        demo_mode()
    else:
        sim = TFBremspruefstandSim()
        sim.start()
