"""Cross-platform serial-port discovery for the ESP32-S3 controller."""

from __future__ import annotations

from serial.tools import list_ports

ESPRESSIF_VID = 0x303A


def default_controller_port() -> str:
    """Return the most likely Espressif port, or an empty string if ambiguous."""
    ports = list(list_ports.comports())
    for port in ports:
        text = " ".join(filter(None, (port.description, port.manufacturer, port.product))).lower()
        if port.vid == ESPRESSIF_VID or "espressif" in text or "usb jtag" in text:
            return port.device
    return ports[0].device if len(ports) == 1 else ""
