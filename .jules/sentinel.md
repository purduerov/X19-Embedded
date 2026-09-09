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

## 2026-09-08 - Prevent struct.unpack Denial of Service on Truncated Payloads
**Vulnerability:** Python scripts parsing binary SIL network payloads (`tests/sil_companion_bridge/sil_protocol.py`) lacked exact payload length validations prior to using `struct.unpack`.
**Learning:** `struct.unpack` enforces strict input sizes. A malformed or truncated CAN frame sent over the network (e.g., from a noisy bus) can immediately crash the topside bridge and UI with a `struct.error`, creating a Denial of Service.
**Prevention:** Python scripts parsing binary CAN/network payloads (e.g., can_sniffer.py, sil_protocol.py) must explicitly validate the byte buffer length before calling struct.unpack to gracefully handle malformed frames without crashing.
## 2025-02-28 - Incomplete Cryptographic Upgrade in Protocol Boundary
**Vulnerability:** Weak Firmware Validation using CRC32 in Bootloader
**Learning:** When upgrading a checksum (like CRC32) to a cryptographic hash (like SHA-256) in a cross-boundary communication payload (e.g., a CAN FD message sent from a Python script to an STM32 C firmware), both the sender and the receiver must be updated to expect the new payload size and format. If the counterpart source code (in this case, the STM32 bootloader firmware) is not present in the repository, updating only the Python script will break the flashing system entirely.
**Prevention:** Always verify the existence and modify the receiving end of a protocol message when changing its size or structure. If the counterpart cannot be updated, do not commit partial changes that break backward compatibility.
