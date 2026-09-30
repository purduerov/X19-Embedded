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

## 2026-09-19 - Untrusted Field Length Buffer Over-read in C SIL Bridge
**Vulnerability:** The `sil_bridge_server.c` extracted the `len` field from an incoming, untrusted TCP packet (`sil_can_packet_t`) and passed it directly to `can_send()` and `memcpy()` without validating it against the protocol maximum (64 bytes).
**Learning:** Network stream parsing logic correctly validated the struct size but implicitly trusted the nested `len` field populated by the client, creating a critical buffer over-read risk on the C side.
**Prevention:** Always validate extracted length fields against protocol bounds (e.g., `<= 64`) before using them in function calls or memory operations when parsing untrusted binary network streams.

## 2026-09-21 - Python SIL Unpack Buffer Over-read Denial of Service
**Vulnerability:** Python scripts parsing binary SIL network payloads (`tests/sil_companion_bridge/sil_protocol.py`) lacked exact payload length validations prior to using `struct.unpack` array slicing.
**Learning:** Network protocols extracted lengths from an untrusted SIL CAN header and used it directly in a slice without constraint checking (`data[:length]`), exposing the system to exceptions/crash if the length exceeds the expected standard size.
**Prevention:** Python scripts parsing binary CAN/network payloads must explicitly validate the length field against the maximum allowable size (e.g. 64 bytes) to safely handle and reject malicious packets without crashing the runtime.
## 2026-09-24 - Stream Parsing Desynchronization via Silent Clamping
**Vulnerability:** The SIL bridge server (`tests/sil_bridge_server.c`) silently clamped incoming CAN frame payloads that exceeded 64 bytes instead of rejecting them.
**Learning:** Clamping length variables during stream parsing causes data desynchronization. If a corrupted header reports a length greater than the maximum, reading only the clamped amount leaves the remainder of the invalid payload in the stream, which is then incorrectly parsed as the next packet's header.
**Prevention:** When parsing stream-based network protocols, validate length headers strictly. If a length exceeds protocol bounds, explicitly drop the malformed packet by advancing the stream buffer and continuing to properly resynchronize the stream.
## 2026-09-24 - Python SIL Dashboard Reader Died on a Desynchronized Stream
**Vulnerability:** `SilDashboardClient._rx_loop` (`tests/sil_dashboard/sil_dashboard_client.py`) carved fixed `SIL_PACKET_SIZE` windows out of the TCP stream and handed each one straight to `unpack_sil_can_frame` with no magic-header check. A window that did not start on a header raised `ValueError`, which the loop's own `except Exception: break` turned into a dead reader: the thread exited, `connected` went False, and the operator lost the entire telemetry feed until they reconnected by hand. One misaligned window was sufficient. A second, narrower case had the same outcome: a well-formed frame whose *payload* was too short to decode raised out of the per-ID `unpack` call.
**Learning:** A broad `except` around a receive loop turns a single malformed packet into loss of the whole stream, so the severity is "connection dropped", not "process crashed" -- and the reader thread exiting is what an operator actually sees. Header validation alone does not cover the case where the header is valid and the payload is not.
**Prevention:** In a continuous binary stream, validate the synchronization marker *before* consuming a window, and on mismatch slide the buffer one byte and retry rather than dropping a whole window. Guard each payload decoder against its own length check as well, so a short payload is skipped instead of unwinding the loop. Wrap the body in `try...except ValueError` as a last resort, but do not rely on it: it converts a crash into a disconnect, which is a better failure, not a fixed one. Keep any buffer bounded, as `tests/sil_companion_bridge/pi_core_sil_bridge.py` already does.
## 2026-10-25 - Python SIL pi_core_sil_bridge Unpack ValueError Denial of Service
**Vulnerability:** The Python `tests/sil_companion_bridge/pi_core_sil_bridge.py` script unpacked CAN payloads (`NavTelemetry.unpack`, `EnvTelemetry.unpack`, `PowerTelemetry.unpack`) directly inside the `_process_can_frame` handler without catching `ValueError` exceptions raised by malformed or truncated CAN frames, unlike `sil_dashboard_client.py`.
**Learning:** `struct.unpack` enforces strict input sizes. A malformed CAN frame or network packet would cause the unpacking to raise a `ValueError`, which was uncaught, crashing the `pi_core_sil_bridge` daemon and creating a Denial of Service (DoS) for the operator's telemetry feed.
**Prevention:** In Python network parsing scripts, always wrap unpacking logic that extracts data from untrusted network stream buffers (like CAN payloads over a TCP bridge) in a `try...except ValueError` block. Safely handle the exception by dropping the malformed packet rather than allowing the runtime to crash.
