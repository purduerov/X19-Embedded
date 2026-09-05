## 2026-03-30 - Generator Expression for String Joining in Telemetry Sniffer
**Learning:** In Python `str.join()`, using a generator expression avoids allocating an intermediate list in memory compared to a list comprehension, although for very small fixed-size sequences (N=8) execution time overhead is comparable or slightly higher due to generator frame instantiation.
**Action:** Use generator expressions when formatting sequences with `str.join()` to eliminate temporary heap allocations when processing continuous streams.
