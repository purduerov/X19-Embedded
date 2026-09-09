## 2026-09-07 - Host Python Tool CAN Payload Validation
**Vulnerability:** `can_sniffer.py` unpacked CAN payloads assuming valid structures, risking index errors (`struct.error: unpack requires a buffer of...`). `can_flash.py` progressed without awaiting ACK replies.
**Learning:** In CAN networks, noisy buses or faulty nodes can easily generate malformed or truncated frames. The python tools were completely blind to this and failed insecurely.
**Prevention:** Add exact payload length validations based on `rov_can_protocol.h` (`len(data) == X`) before decoding and enforce state progression based on strict `NACK` and `ACK` timeouts.

## 2026-09-07 - Silent Data Truncation in CAN Interface
**Vulnerability:** A classic CAN transmit function silently truncated payload lengths to 8 bytes if given a larger payload (e.g., from a CAN FD capable node/caller).
**Learning:** Silently clamping length variables instead of throwing an error can cause hard-to-detect data loss vulnerabilities, where the sender assumes the full payload was transmitted.
**Prevention:** Validate constraints and explicitly reject out-of-bounds parameters (return an error code or false) rather than silently truncating data, especially when interfacing legacy hardware with newer abstractions.

## 2026-09-07 - Prevent struct.pack Denial of Service on Unbounded Values
**Vulnerability:** A `struct.error` could be raised, causing a Denial of Service, when attempting to pack integer values that exceed the format string limits (e.g., packing a value > 65535 using `<H`).
**Learning:** Python `struct.pack` enforces strict bounds. Unvalidated input affecting loops or mathematical derivations used in packing must be checked beforehand.
**Prevention:** Always validate and bound check derived integer values (like calculating chunk counts from file size) before passing them to `struct.pack` if they are constrained by type limits.

## 2026-09-07 - SIL Protocol Structural Unpacking DoS
**Vulnerability:** The Software-In-the-Loop (SIL) TCP bridge unmarshaled raw bytes using `struct.unpack` without prior length checks, leading to unhandled `struct.error` exceptions.
**Learning:** Network boundaries parsing binary protocols must never assume the underlying transport provides the exact number of bytes required by a fixed struct definition.
**Prevention:** Always validate `len(buffer) >= REQUIRED_BYTES` before passing data to struct decoding functions to handle truncated packets gracefully with standard ValueErrors.
