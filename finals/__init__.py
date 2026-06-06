"""BrainHack 2026 finals — C2-laptop swarm-challenge package.

All mission code runs on the C2 laptop, controlling Highgreat HULA drones over
Wi-Fi via the pyhulax SDK, with detection on laptop-streamed video frames.

Entry point: ``python -m finals.main --profile {mock,sitl,replay,bench,real}``

Start here when picking up a session: finals/docs/module_map.md
Architecture + roadmap reference:    docs/finals/README.md (repo docs tree)

Root qualifier files are imported where proven (drone_control.py,
get_position_with_task.py) and NEVER modified from this package.
"""
