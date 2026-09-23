def command_speed(travelled, target, speed):
    """Return forward speed until the measured displacement reaches the target."""
    return speed if travelled < target else 0.0
