"""Iris thrust map: armed spin and the hover motor constant."""

MASS_KG: float = 1.5
G_MPS2: float = 9.81
MAX_SPIN_RAD_S: float = 1100.0


def spin_rad_s(command: float) -> float:
    """Armed motor spin for a normalized command, clamped at max spin."""
    return min(command * 1000.0 + 100.0, MAX_SPIN_RAD_S)


def motor_constant(hover_command: float) -> float:
    """motorConstant so four motors at spin(h) produce the vehicle weight."""
    spin = spin_rad_s(hover_command)
    return (MASS_KG * G_MPS2) / (4.0 * spin * spin)
