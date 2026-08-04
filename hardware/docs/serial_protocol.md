# Phase A — Serial Protocol (Arduino ↔ Raspberry Pi)

*The contract between the fast motor board (Arduino) and the orchestrator (Pi). Everything in Phase B/C rides on this, so it's worth getting simple and robust.*

## Design decisions (the "why")

- **ASCII, line-based.** One message = one `\n`-terminated line of space-separated fields. Binary would be a few bytes smaller, but ASCII means you can open the Arduino Serial Monitor, type a command by hand, and read the reply — priceless while debugging. At 50–100 Hz with 7 numbers, ASCII is not the bottleneck.
- **Request → reply, one reply per request.** The Pi drives the timing. Every command the Pi sends gets exactly one `s ...` state line back. This keeps the loop synchronous and easy to reason about (no guessing whether a stray line is old).
- **Angles in degrees on the wire.** The firmware stays in degrees (what hobby servos speak); the Pi driver converts to radians and applies calibration. Keep unit conversion in *one* place (the Pi), not split across both.
- **State is always the *measured* value.** The `s` reply reports what the **encoders** read right now — never the last commanded target. This is what makes the data honest for imitation learning.

## Messages: Pi → Arduino

| Line | Meaning |
|---|---|
| `S d0 d1 d2 d3 d4 d5 g` | **Set** target angles (degrees) for joints 0–5 and gripper `g` (0–100). Arduino updates targets, then replies with current measured state. |
| `Q` | **Query** only — read encoders, don't move. Replies with state. |
| `H` | Go to the **home** pose (defined in firmware). |
| `E` | **Enable** torque (attach/energize servos). |
| `X` | Relax / **e-stop** — de-energize servos so the arm is safe and (if backdrivable) hand-movable. |

## Messages: Arduino → Pi

| Line | Meaning |
|---|---|
| `s d0 d1 d2 d3 d4 d5 g t_ms` | **State**: measured joint angles (deg), gripper (0–100), and the Arduino's `millis()` timestamp `t_ms`. Sent once in reply to every command. |
| `# ...` | A comment / log line (e.g. `# ready` banner at boot). The Pi ignores any line starting with `#`. |
| `! ...` | An **error** (e.g. `! bad_cmd`, `! out_of_range`). The Pi should log these. |

## Example exchange

```
# ready                        (Arduino, on boot)
Q                              (Pi asks for state)
s 90.1 45.2 120.0 88.7 10.3 0.5 12 10432
S 95 45 120 90 10 0 20         (Pi commands new targets)
s 90.3 45.1 120.0 88.9 10.2 0.6 12 10461   (reply: still moving toward target)
```

## Conventions & safety

- **Rate:** the Pi polls at a fixed period (start at 20 Hz = every 50 ms; raise to 50–100 Hz once stable). The Arduino should reply in well under that period.
- **Joint limits** live in the firmware (hard safety) *and* the Pi driver (soft, friendlier errors). Firmware clamps out-of-range targets and emits `! out_of_range`.
- **Framing robustness:** the Pi reads full lines and discards partial/garbled ones. A one-byte XOR checksum can be added later as an 8th field if you see corruption — noted, not needed to start.
- **Baud:** 115200 (reliable, fast enough).
