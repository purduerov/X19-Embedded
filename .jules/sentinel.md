## 2024-05-01 - Host Python Tool CAN Payload Validation
**Vulnerability:** `can_sniffer.py` unpacked CAN payloads assuming valid structures, risking index errors (`struct.error: unpack requires a buffer of...`). `can_flash.py` progressed without awaiting ACK replies.
**Learning:** In CAN networks, noisy buses or faulty nodes can easily generate malformed or truncated frames. The python tools were completely blind to this and failed insecurely.
**Prevention:** Add exact payload length validations based on `x19_can_protocol.h` (`len(data) == X`) before decoding and enforce state progression based on strict `NACK` and `ACK` timeouts.
