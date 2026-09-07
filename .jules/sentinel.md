## 2024-05-14 - Unverified CAN FD Data Trust
**Vulnerability:** The firmware deserialized thruster PWM commands from the CAN FD bus but did not verify the logical bounds of the data before passing it to subsequent logic. If a malformed or malicious packet was injected into the CAN bus, the thrusters could be commanded with invalid PWM intervals leading to hardware damage or unpredictable behavior.
**Learning:** Never inherently trust telemetry or command data received from a shared bus like CAN FD, even if it comes from an internal source. Deserialization is only the first step; logical bounds checking must be applied immediately.
**Prevention:** Implement immediate bounds checking at the point of unpack/deserialization. In this case, `x19_can_unpack_thruster_cmd` now validates the PWM values against `X19_PWM_MIN_US` and `X19_PWM_MAX_US`. If validation fails, it zeroes the output to fail securely.
## 2024-05-30 - Prevent struct.pack Denial of Service on Unbounded Values
**Vulnerability:** A `struct.error` could be raised, causing a Denial of Service, when attempting to pack integer values that exceed the format string limits (e.g., packing a value > 65535 using `<H`).
**Learning:** Python `struct.pack` enforces strict bounds. Unvalidated input affecting loops or mathematical derivations used in packing must be checked beforehand.
**Prevention:** Always validate and bound check derived integer values (like calculating chunk counts from file size) before passing them to `struct.pack` if they are constrained by type limits.
