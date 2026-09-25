# SIL Dashboard and Fake-CAN Stimulus Design

Date: 2026-09-25
Status: Approved design, pending written-spec review

## Context

The host SIL and native node tests are passing, but the interactive dashboard
and external stimulus tools are not yet reliable enough to be treated as a
complete vehicle test surface.

Verified gaps:

- The dashboard sends a control frame only when a control changes. Node 2's
  100 ms heartbeat then expires and returns PWM outputs to neutral.
- `tools/can_stimulus.py` and `tools/node2_stimulus.py` are untracked local
  files rather than supported repository tools.
- The local Node 2 stimulus script references a missing `args.port` and fails
  in SIL auto mode.
- The vehicle-wide stimulus script can print success while Node 3 power
  decoding fails, because warnings do not affect the process status.
- Target readiness tests remain incomplete because generated startup, linker,
  pin, BSP, FDCAN, and hardware sensor integration is absent.

This change is host-first. It will not invent target pin mappings or claim
that host SIL results prove physical hardware readiness.

## Goals

1. Make the dashboard sustain pilot commands at 20 Hz and fail safe to
   neutral on stop, disconnect, transport error, or emergency break.
2. Provide one maintained vehicle-wide fake-CAN stimulus tool for Nodes 1, 2,
   and 3, with a compatibility wrapper for the existing Node 2 entry point.
3. Make every stimulus smoke test fail with a nonzero process status when a
   required observation is missing or incorrect.
4. Add automated host-side tests that exercise the native C SIL server through
   the same protocol used by the dashboard and stimulus tools.
5. Keep the scope and limitations of the SIL explicit in documentation and CI.

## Non-goals

- Implementing or guessing STM32C542 startup code, linker scripts, GPIO
  assignments, FDCAN filters, or physical sensor transactions.
- Claiming that SIL models FDCAN electrical timing, arbitration, ACK errors,
  retransmission, STM32 timer/BDTR behavior, or hardware fault confinement.
- Replacing the existing native CTest suite with a Python-only test layer.

## Architecture

### Canonical stimulus tool

`tools/can_stimulus.py` will be the single implementation. It will provide:

- A SIL TCP backend using the existing `sil_protocol` framing.
- Optional python-can and pyserial hardware adapters.
- Node 1 environment/leak monitoring and fault injection.
- Node 2 arming, PWM, depth, solenoid, and emergency checks.
- Node 3 power telemetry monitoring and fault signaling.
- Raw frame injection and bus sniffing.

`tools/node2_stimulus.py` will be a thin compatibility wrapper that delegates
to the canonical tool. Protocol structures and backend implementations will
not be duplicated between the two entry points.

The tool will validate frame lengths, PWM bounds, solenoid masks, and required
signatures before transmitting. Every check will return a boolean or raise a
controlled test failure; warnings will not be converted into successful smoke
results.

### Dashboard control worker

`SilDashboardClient` will own an explicit control state machine:

```text
DISCONNECTED -> STOPPED -> ARMED -> RUNNING
      ^            |          |         |
      +------------+----------+---------+
                 STOP / ESTOP
```

- `DISCONNECTED`: no CAN frames are sent.
- `STOPPED`: PWM is neutral and solenoids are zero.
- `ARMED`: the transport is connected, but no non-neutral command is active.
- `RUNNING`: the last accepted command is resent every 50 ms (20 Hz).
- `ESTOP`: sends the authorized emergency frame, forces neutral, and latches
  until the engine is restarted.

The worker sends neutral on every exit path: explicit stop, emergency break,
socket error, worker exception, disconnect, and stale/invalid command state.
The dashboard will expose the current state and allow an explicit All-Stop
control.

### Data flow

```text
Streamlit controls
      |
      v
SilDashboardClient control worker (20 Hz)
      |
      v
SIL TCP CAN frame
      |
      v
sil_bridge_server
      |
      v
Node 1 / Node 2 / Node 3 native application logic
      |
      v
Mock output snapshot and telemetry frames
```

The stimulus CLI uses the same SIL backend and framing, so its checks exercise
native C application behavior rather than a separate Python model.

## Failure handling

- Missing optional hardware dependencies are errors only when the
  corresponding hardware mode is selected.
- SIL startup errors report the expected build command and searched binary
  paths.
- Telemetry waits have finite timeouts and required-frame assertions.
- Emergency frames require `0xAA 0x55`; eFuse alerts require `0xEF 0x01`.
- Node 2 arming verification must observe both the neutral interval and the
  post-arming transition.
- Emergency verification must observe output-status readback, not merely a
  successful socket write.
- Process exit status is nonzero for any failed or incomplete smoke check.

## Test strategy

Add a host-side Python test package that runs against the compiled native
`sil_bridge_server`:

1. Protocol and backend framing round-trip.
2. Dashboard sustained 20 Hz control and neutral-on-stop/error behavior.
3. Node 1 environmental telemetry and leak response.
4. Node 2 arming, PWM response, solenoid interlock, and emergency cutoff.
5. Node 3 power telemetry and fault signaling.
6. CLI success and failure exit statuses.
7. `node2_stimulus.py` compatibility-wrapper behavior.

Retain the existing native CTest suite as the authoritative application and
driver regression suite.

CI will:

- Build the native SIL server.
- Run the existing non-target CTest suite.
- Run the new Python SIL stimulus tests with an explicit
  `X19_SIL_SERVER` path.
- Run dashboard syntax/import checks separately from required SIL tests so an
  optional UI dependency cannot silently skip coverage.
- Keep target readiness checks explicitly non-blocking and documented.

## Acceptance criteria

The work is complete only when:

- A dashboard command remains active under the 20 Hz worker and returns to
  neutral on stop, disconnect, or error.
- All three node stimulus flows pass against the native SIL server.
- Missing or incorrect stimulus observations produce a nonzero exit status.
- The canonical tool and compatibility wrapper are committed in the
  repository.
- Native and Python tests pass from a clean build.
- Target-hardware limitations remain explicitly documented.

## Risks and mitigations

- **Threaded Streamlit state races:** keep command state ownership in the
  client worker and protect transitions with the existing client lock.
- **Port conflicts during tests:** allocate a free TCP port per test and pass
  it explicitly to the server and client.
- **Protocol drift:** import the canonical SIL protocol module rather than
  maintaining fallback packet structures in the tools.
- **False-positive smoke output:** aggregate named checks into one result and
  return nonzero from `main()`.
- **Scope creep into hardware bring-up:** keep target readiness failures
  visible and out of the host pass criteria.
