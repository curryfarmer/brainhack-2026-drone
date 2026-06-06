"""MockAdapter — the scriptable FlightAdapter test double.

Planned surface (S3):
- Implements finals.flight.adapter.FlightAdapter. Instant success by default
  with configurable latency_s; records every call in .calls for assertions.
- Scriptable failures: fail_on={"move": FlightTimeout("...")} or
  fail_at="move:3" (3rd move raises); battery decay curve; telemetry freeze
  after N seconds — so every guard and abort path is unit-testable.
- Maintains a simulated 2D pose + yaw integrated from executed actions via
  finals.flight.dead_reckon (shared math — the mock doubles as the
  dead-reckoning reference implementation).

Derives from: nothing external — it IS the test double the whole pyramid
stands on (used by tests for agent, orchestrator, guards, phases).

STUB — session S3.
"""
from __future__ import annotations

_STUB = "finals.flight.mock_adapter: session S3 — see finals/docs/module_map.md"


class MockAdapter:  # implements finals.flight.adapter.FlightAdapter in S3
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
