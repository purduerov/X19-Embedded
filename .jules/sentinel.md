## 2024-05-14 - Unverified CAN FD Data Trust
**Vulnerability:** The firmware deserialized thruster PWM commands from the CAN FD bus but did not verify the logical bounds of the data before passing it to subsequent logic. If a malformed or malicious packet was injected into the CAN bus, the thrusters could be commanded with invalid PWM intervals leading to hardware damage or unpredictable behavior.
**Learning:** Never inherently trust telemetry or command data received from a shared bus like CAN FD, even if it comes from an internal source. Deserialization is only the first step; logical bounds checking must be applied immediately.
**Prevention:** Implement immediate bounds checking at the point of unpack/deserialization. In this case, `x19_can_unpack_thruster_cmd` now validates the PWM values against `X19_PWM_MIN_US` and `X19_PWM_MAX_US`. If validation fails, it zeroes the output to fail securely.

## 2026-09-07 - Host Python Tool CAN Payload Validation
**Vulnerability:** `can_sniffer.py` unpacked CAN payloads assuming valid structures, risking index errors (`struct.error: unpack requires a buffer of...`). `can_flash.py` progressed without awaiting ACK replies.
**Learning:** In CAN networks, noisy buses or faulty nodes can easily generate malformed or truncated frames. The python tools were completely blind to this and failed insecurely.
**Prevention:** Add exact payload length validations based on `rov_can_protocol.h` (`len(data) == X`) before decoding and enforce state progression based on strict `NACK` and `ACK` timeouts.

## 2026-09-07 - Prevent silent truncation of CAN payloads
**Vulnerability:** The `can_send` function in `can_f4.c` silently truncated payloads larger than 8 bytes by setting `len = 8`, which could cause incomplete packet transmission.
**Learning:** In hardware abstraction layers, operations that exceed hardware limits (like payload size) must explicitly fail rather than trying to auto-correct, as auto-correction often leads to silent data corruption or loss.
**Prevention:** When interfacing with hardware constraints (like 8-byte CAN frames), validate input sizes and return an explicit error code or boolean `false` rather than clamping/truncating values silently.
