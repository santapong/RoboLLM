"""Metric A4 PDF/SVGs. SVG and PDF share the same millimetre coordinates."""

from __future__ import annotations

import base64
import io
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

MM = 72 / 25.4
DICT = cv2.aruco.DICT_5X5_100
# Marker lower-left coordinates relative to the task-grid origin, in metres.
MAT_MARKERS = {
    50: (-0.030, -0.030),
    51: (0.190, -0.030),
    52: (0.190, 0.130),
    53: (-0.030, 0.130),
}
MAT_MARKER_SIZE = 0.020


def board():
    return cv2.aruco.CharucoBoard(
        (7, 5), 0.030, 0.022, cv2.aruco.getPredefinedDictionary(DICT)
    )


class Sheet:
    def __init__(self, pdf, destination):
        self.pdf, self.destination = pdf, destination
        self.svg = ET.Element(
            "svg",
            xmlns="http://www.w3.org/2000/svg",
            width="297mm",
            height="210mm",
            viewBox="0 0 297 210",
        )
        self.rect(0, 0, 297, 210, fill="white", stroke="none")

    def rect(self, x, y, w, h, fill="none", stroke="#171c22", width=0.2):
        self.pdf.setLineWidth(width * MM)
        if stroke != "none":
            self.pdf.setStrokeColor(stroke)
        if fill != "none":
            self.pdf.setFillColor(fill)
        self.pdf.rect(
            x * MM,
            y * MM,
            w * MM,
            h * MM,
            stroke=int(stroke != "none"),
            fill=int(fill != "none"),
        )
        ET.SubElement(
            self.svg,
            "rect",
            x=str(x),
            y=str(210 - y - h),
            width=str(w),
            height=str(h),
            fill=fill,
            stroke=stroke,
            **{"stroke-width": str(width)},
        )

    def text(self, x, y, text, size=9, color="#171c22", bold=False):
        self.pdf.setFillColor(color)
        self.pdf.setFont("Helvetica-Bold" if bold else "Helvetica", size)
        self.pdf.drawString(x * MM, y * MM, text)
        el = ET.SubElement(
            self.svg,
            "text",
            x=str(x),
            y=str(210 - y),
            fill=color,
            **{
                "font-family": "Arial, sans-serif",
                "font-size": str(size / MM),
                "font-weight": "700" if bold else "400",
            },
        )
        el.text = text

    def line(self, x1, y1, x2, y2, width=0.2, color="#171c22"):
        self.pdf.setStrokeColor(color)
        self.pdf.setLineWidth(width * MM)
        self.pdf.line(x1 * MM, y1 * MM, x2 * MM, y2 * MM)
        ET.SubElement(
            self.svg,
            "line",
            x1=str(x1),
            y1=str(210 - y1),
            x2=str(x2),
            y2=str(210 - y2),
            stroke=color,
            **{"stroke-width": str(width)},
        )

    def image(self, img, x, y, w, h):
        ok, encoded = cv2.imencode(".png", img)
        if not ok:
            raise RuntimeError("pattern encoding failed")
        data = encoded.tobytes()
        self.pdf.drawImage(
            ImageReader(io.BytesIO(data)), x * MM, y * MM, width=w * MM, height=h * MM
        )
        ET.SubElement(
            self.svg,
            "image",
            x=str(x),
            y=str(210 - y - h),
            width=str(w),
            height=str(h),
            href="data:image/png;base64," + base64.b64encode(data).decode(),
        )

    def marker(self, identifier, x, y, size=20):
        img = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(DICT), identifier, 400
        )
        self.image(img, x, y, size, size)

    def finish(self):
        self.destination.write_text(ET.tostring(self.svg, encoding="unicode"))
        self.pdf.showPage()


def scale(s, x, y):
    s.line(x, y, x + 100, y, 0.35)
    for a in range(0, 101, 10):
        s.line(x + a, y - 1, x + a, y + 1, 0.25)
    s.text(x + 32, y + 3, "100 mm - verify with a ruler", 8)


def generate(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(
        str(out / "elexhub-reference-sheets.pdf"),
        pagesize=(297 * MM, 210 * MM),
        invariant=1,
    )
    pdf.setTitle("Elexhub metric reference sheets - print at actual size")
    s = Sheet(pdf, out / "camera-calibration.svg")
    s.text(12, 198, "RoboLLM / Elexhub - camera calibration", 15, bold=True)
    s.text(
        12,
        188,
        "A4 landscape | 7 x 5 squares | square 30 mm | marker 22 mm | DICT_5X5_100",
        9,
    )
    s.image(board().generateImage((2100, 1500)), 43.5, 30, 210, 150)
    scale(s, 98.5, 15)
    s.text(
        12,
        5,
        "PRINT AT 100% / ACTUAL SIZE. Disable fit-to-page. Keep flat; measure a square before calibration.",
        8,
    )
    s.finish()
    s = Sheet(pdf, out / "workspace-mat.svg")
    s.text(58.5, 200, "RoboLLM / Elexhub - tabletop coordinates", 13, bold=True)
    ox, oy = 58.5, 45
    for identifier, (x, y) in MAT_MARKERS.items():
        s.marker(identifier, ox + x * 1000, oy + y * 1000, 20)
    for i in range(19):
        s.line(
            ox + i * 10,
            oy,
            ox + i * 10,
            oy + 120,
            0.25 if i % 3 == 0 else 0.10,
            "#77828c" if i % 3 == 0 else "#bfc6cb",
        )
    for j in range(13):
        s.line(
            ox,
            oy + j * 10,
            ox + 180,
            oy + j * 10,
            0.25 if j % 3 == 0 else 0.10,
            "#77828c" if j % 3 == 0 else "#bfc6cb",
        )
    for i in range(18):
        s.text(ox + i * 10 + 3, oy - 5, chr(65 + i), 7)
    for j in range(12):
        s.text(ox - 7, oy + j * 10 + 3, str(j + 1), 7)
    s.text(ox, 174, "180 x 120 mm workspace | 10 mm grid | +Z above the paper", 9)
    s.text(ox + 2, oy + 2, "O", 7, bold=True)
    s.text(ox + 157, oy + 2, "+X", 8, bold=True)
    s.text(ox + 2, oy + 114, "+Y", 8, bold=True)
    scale(s, 98.5, 24)
    s.text(
        58.5,
        10,
        "Print actual size. Register this origin to the robot base before motion.",
        8,
    )
    s.finish()
    s = Sheet(pdf, out / "tracking-markers.svg")
    s.text(12, 198, "RoboLLM / Elexhub - tool and object markers", 15, bold=True)
    s.text(
        12,
        187,
        "Cut with a white border. Measure the black marker edge; store the marker-to-tool offset.",
        10,
    )
    s.marker(60, 35, 130, 20)
    s.text(70, 143, "Tool marker: ID 60 / black edge 20 mm", 12, bold=True)
    s.text(70, 131, "Attach to a rigid, visible bracket. Recalibrate if it moves.", 10)
    s.marker(70, 37.5, 85, 15)
    s.text(70, 96, "Object marker: ID 70 / black edge 15 mm", 12, bold=True)
    s.text(
        70, 84, "Optional marker-first box pose. Record its placement on the box.", 10
    )
    s.text(
        35,
        55,
        "These IDs are distinct from the workspace and camera-calibration boards.",
        10,
    )
    scale(s, 98.5, 25)
    s.finish()
    pdf.save()

    pdf = canvas.Canvas(
        str(out / "elexhub-wiring-and-measurements.pdf"),
        pagesize=(297 * MM, 210 * MM),
        invariant=1,
    )
    pdf.setTitle("Elexhub wiring concept and bench measurement worksheet")
    s = Sheet(pdf, out / "wiring.svg")
    s.text(12, 197, "RoboLLM / Elexhub - wiring and power", 16, bold=True)
    s.text(
        12,
        186,
        "DESIGN PENDING COMPONENT RATINGS - complete the worksheet before connecting servo power.",
        9,
        color="#984210",
        bold=True,
    )

    def box(x, y, title, sub):
        s.rect(x, y, 57, 24, fill="#f3f5f6")
        s.text(x + 3, y + 15, title, 11, bold=True)
        s.text(x + 3, y + 5, sub, 8)

    for x, title, sub in [
        (12, "Raspberry Pi", "USB host"),
        (84, "Arduino Mega", "Limits / watchdog"),
        (156, "PCA9685", "PWM output"),
        (228, "Six servos", "Five joints + gripper"),
    ]:
        box(x, 140, title, sub)
    for x, label in [(69, "USB"), (141, "I2C"), (213, "PWM")]:
        s.line(x, 152, x + 15, 152, 0.4, "#245b88")
        s.text(x + 2, 156, label, 7)
    for x, title, sub in [
        (12, "Protected battery", "Voltage / current: unset"),
        (84, "Fuse + cutoff", "DC rating: unset"),
        (156, "Regulator / BEC", "Output V / A: unset"),
        (228, "Power distribution", "Rated wire; shared GND"),
    ]:
        box(x, 77, title, sub)
    for x in [69, 141, 213]:
        s.line(x, 89, x + 15, 89, 0.4, "#a8392e")
    s.line(256.5, 101, 256.5, 140, 0.4, "#a8392e")
    s.text(260, 118, "Servo rail", 8)
    s.text(
        12,
        125,
        "Mega D20 -> SDA     D21 -> SCL     5 V -> VCC (logic only)     GND -> GND",
        10,
    )
    s.text(
        12,
        114,
        "Mega D22 -> OE, with verified default-disable pull-up. All signal grounds share a reference.",
        10,
    )
    s.text(
        12,
        59,
        "1. Never power servo motors from Mega/Pi 5 V. Check the breakout's terminal and trace ratings.",
        10,
    )
    s.text(
        12,
        48,
        "2. OE disables PWM; it does not disconnect motor power. Use the physical DC-rated cutoff.",
        10,
    )
    s.text(
        12,
        37,
        "3. Check regulator input range against the fully charged AND discharged battery voltages.",
        10,
    )
    s.text(
        12,
        26,
        "4. Calibrate one unloaded servo at a time. Confirm polarity and signal pin labels on your actual board.",
        10,
    )
    s.text(
        12,
        9,
        "Sources: nxp.com/docs/en/data-sheet/PCA9685.pdf | learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up",
        7,
    )
    s.finish()
    s = Sheet(pdf, out / "measurement-worksheet.svg")
    s.text(12, 198, "RoboLLM / Elexhub - measurement record", 16, bold=True)
    s.text(
        12,
        186,
        "Date / operator: __________________________    Photo / drawing reference: __________________________",
        10,
    )
    labels = [
        "Battery cells / capacity / min-full V",
        "BMS / discharge rating / charger",
        "Regulator model / input V / output V-A",
        "Fuse / wire / cutoff ratings",
        "Arm base / camera / mat position",
    ]
    for i, label in enumerate(labels):
        s.text(
            12,
            171 - i * 12,
            label + ": __________________________________________________________",
            10,
        )
    columns = [12, 37, 84, 113, 144, 175, 218, 285]
    headers = [
        "Actuator",
        "Servo model",
        "Link mm",
        "Axis",
        "Limits deg",
        "Pulse us",
        "Mass g",
    ]
    top = 103
    rowh = 11
    for x in columns:
        s.line(x, top - 7 * rowh, x, top)
    for i in range(8):
        s.line(12, top - i * rowh, 285, top - i * rowh)
    for x, title in zip(columns, headers):
        s.text(x + 2, top - 7, title, 8, bold=True)
    for i, name in enumerate(
        ["Base", "Shoulder", "Elbow", "Wrist pitch", "Wrist roll", "Gripper"]
    ):
        s.text(14, top - (i + 1) * rowh - 7, name, 8)
    s.text(
        12,
        15,
        "Also record: neutral, direction, safe speed, voltage range, stall current, gripper opening, cutoff and watchdog evidence.",
        8,
    )
    s.text(
        12,
        7,
        "All supplied CAD dimensions are PROVISIONAL until replaced with this arm's measurements.",
        8,
        bold=True,
    )
    s.finish()
    pdf.save()
    return [
        out / "elexhub-reference-sheets.pdf",
        out / "elexhub-wiring-and-measurements.pdf",
    ]
