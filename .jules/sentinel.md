## 2026-09-04 - [Buffer Overflow Prevention in CAN Packing]
**Vulnerability:** Found buffer overflow vulnerability where CAN packet serialization routines assumed destination buffer size implicitly, leading to out-of-bounds writes if the caller supplied a smaller buffer.
**Learning:** The *len pointer was being used as an OUT parameter only. In low-level C networking, buffer capacity should be an IN/OUT parameter to protect memory.
**Prevention:** Added explicit capacity checks on all x19_can_pack_* functions that verify *len is greater than or equal to sizeof(target_struct) before writing with memcpy.
