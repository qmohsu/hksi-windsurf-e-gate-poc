"""
I/O Module: Network, serial, message parsing, bounded queues.

Implements Rule 50 (RPi I/O ops) requirements:
- Bounded queues (no unbounded RAM growth)
- Disconnect/reconnect tolerance
- Backpressure handling
"""
