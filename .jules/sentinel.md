## 2024-05-14 - Unverified CAN FD Data Trust
**Vulnerability:** The firmware deserialized thruster PWM commands from the CAN FD bus but did not verify the logical bounds of the data before passing it to subsequent logic. If a malformed or malicious packet was injected into the CAN bus, the thrusters could be commanded with invalid PWM intervals leading to hardware damage or unpredictable behavior.
**Learning:** Never inherently trust telemetry or command data received from a shared bus like CAN FD, even if it comes from an internal source. Deserialization is only the first step; logical bounds checking must be applied immediately.
**Prevention:** Implement immediate bounds checking at the point of unpack/deserialization. In this case, `x19_can_unpack_thruster_cmd` now validates the PWM values against `X19_PWM_MIN_US` and `X19_PWM_MAX_US`. If validation fails, it zeroes the output to fail securely.
## 2024-05-24 - Silent Data Truncation in CAN Interface
**Vulnerability:** A classic CAN transmit function silently truncated payload lengths to 8 bytes if given a larger payload (e.g., from a CAN FD capable node/caller).
**Learning:** Silently clamping length variables instead of throwing an error can cause hard-to-detect data loss vulnerabilities, where the sender assumes the full payload was transmitted.
**Prevention:** Validate constraints and explicitly reject out-of-bounds parameters (return an error code or false) rather than silently truncating data, especially when interfacing legacy hardware with newer abstractions.
