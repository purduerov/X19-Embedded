## 2023-10-27 - [Missing Input Validation on CAN Deserialization]
**Vulnerability:** The CAN FD deserialization functions (e.g., `x19_can_unpack_thruster_cmd`) in `shared/src/x19_can_protocol.c` blindly copy memory from the CAN payload directly into internal structs using `memcpy`, without verifying that the values are within safe physical bounds (e.g., PWM limits).
**Learning:** This pattern existed because the architecture optimizes for high-speed, low-overhead communication over CAN FD, prioritizing speed over safety bounds checking at the protocol layer.
**Prevention:** Always validate deseralized data against authoritative safety parameters (defined in `x19_parameters.h`) before returning `X19_OK`, rejecting packets that violate physical or safety constraints.
