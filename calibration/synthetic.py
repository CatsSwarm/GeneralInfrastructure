"""Synthetic velocity commands for iris calibration.

Each case commands one horizontal axis. The other axis and vz stay at 0.
Commands stay within ±MAX_ABS_V_MPS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MAX_ABS_V_MPS: float = 5.0

_STEP_DURATION_S = 4.0
_SINE_DURATION_S = 10.0
_SINE_AMPLITUDE_MPS = 2.0
_CHIRP_DURATION_S = 20.0
_CHIRP_AMPLITUDE_MPS = 1.0


@dataclass(frozen=True)
class SyntheticCase:
    name: str
    duration_s: float
    _axis: str
    _waveform: str
    _amplitude_mps: float
    _frequency_hz: float = 0.0

    def velocity_at(self, t_s: float) -> tuple[float, float, float]:
        if t_s < 0.0 or t_s > self.duration_s:
            raise ValueError(f"t_s {t_s} is outside [0, {self.duration_s}]")
        value = self._sample(t_s)
        if abs(value) > MAX_ABS_V_MPS:
            raise ValueError(f"|v| {abs(value)} exceeds {MAX_ABS_V_MPS} m/s")
        if self._axis == "x":
            return (value, 0.0, 0.0)
        if self._axis == "y":
            return (0.0, value, 0.0)
        raise ValueError(f"unknown axis {self._axis}")

    def _sample(self, t_s: float) -> float:
        if self._waveform == "step":
            return self._amplitude_mps
        if self._waveform == "sine":
            return self._amplitude_mps * math.sin(2 * math.pi * self._frequency_hz * t_s)
        if self._waveform == "chirp":
            phase = 2 * math.pi * (0.1 * t_s + 1.9 * t_s**2 / (2 * 20))
            return self._amplitude_mps * math.sin(phase)
        raise ValueError(f"unknown waveform {self._waveform}")


def synthetic_cases() -> list[SyntheticCase]:
    steps = [
        ("step_vx_p1", "x", 1.0),
        ("step_vx_m1", "x", -1.0),
        ("step_vx_p3", "x", 3.0),
        ("step_vx_m3", "x", -3.0),
        ("step_vx_p5", "x", MAX_ABS_V_MPS),
        ("step_vx_m5", "x", -MAX_ABS_V_MPS),
        ("step_vy_p1", "y", 1.0),
        ("step_vy_m1", "y", -1.0),
        ("step_vy_p3", "y", 3.0),
        ("step_vy_m3", "y", -3.0),
        ("step_vy_p5", "y", MAX_ABS_V_MPS),
        ("step_vy_m5", "y", -MAX_ABS_V_MPS),
    ]
    sines = [
        ("sine_vx_0.2hz", "x", 0.2),
        ("sine_vx_1.0hz", "x", 1.0),
        ("sine_vy_0.2hz", "y", 0.2),
        ("sine_vy_1.0hz", "y", 1.0),
    ]
    cases = [
        SyntheticCase(name, _STEP_DURATION_S, axis, "step", amplitude)
        for name, axis, amplitude in steps
    ]
    cases.extend(
        SyntheticCase(name, _SINE_DURATION_S, axis, "sine", _SINE_AMPLITUDE_MPS, frequency_hz)
        for name, axis, frequency_hz in sines
    )
    cases.append(SyntheticCase("chirp_vx", _CHIRP_DURATION_S, "x", "chirp", _CHIRP_AMPLITUDE_MPS))
    cases.append(SyntheticCase("chirp_vy", _CHIRP_DURATION_S, "y", "chirp", _CHIRP_AMPLITUDE_MPS))
    return cases
