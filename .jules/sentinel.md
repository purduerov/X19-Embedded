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
## 2026-09-25 - Python SIL Dashboard Network Stream DoS
**Vulnerability:** The SIL dashboard client blindly extracted `SIL_PACKET_SIZE` chunks from the TCP stream and decoded them without handling `ValueError` exceptions or validating stream sync headers.
**Learning:** Clamping/extracting network stream chunks without validating sync markers or handling parsing errors allows a single malformed/unaligned network packet to crash the receiver thread, dropping the connection and causing a Denial of Service (DoS) for the operator.
**Prevention:** When parsing continuous binary TCP streams in Python, always validate synchronization magic headers before consuming chunk bytes. If invalid, slide the parsing window by 1 byte to safely resynchronize. Wrap unpacking logic in a `try...except ValueError` block to gracefully drop corrupted payloads without crashing the thread.
