#!/usr/bin/env python3
"""Find the ESP32 on the network, using the firmware's own PING/PONG.

    python tools/find_esp32.py                 # sweep every local subnet
    python tools/find_esp32.py 10.115.35.0/24  # sweep one
    python tools/find_esp32.py 10.115.35.42    # just ask that one host

WHY THIS EXISTS
---------------
The ESP32 takes its address from DHCP and prints it once, at boot:

    Ready. IP: 10.229.5.249

Nothing else ever reports it. That address then gets pasted into
`esp32_ip:=` and stays there -- but it is a LEASE, not a property of the
board. Reboot the router, move to a different network, or let the lease
expire, and the number in the launch file is quietly pointing at nothing.

UDP has no connection, so this fails in total silence: the bridge keeps
sending, `sendto` keeps succeeding, no error is logged anywhere, and the
robot simply does not move. docs/troubleshooting.md calls that "the single
most confusing failure this system has", and it is the reason for this
script.

It uses the protocol's own discovery mechanism rather than ping or ARP:
"PING" -> "PONG" is answered ONLY by something running this firmware, so a
reply proves the board is powered, joined to WiFi, listening on 1234 and
parsing packets. An ICMP ping proves only that some device holds the
address.

If nothing answers, that is informative too -- see the hints it prints.
Nothing here needs ROS.
"""
from __future__ import annotations

import ipaddress
import socket
import subprocess
import sys
import time

PORT = 1234          # FIRMWARE_UDP_PORT in mmr_pkg/esp32_protocol.py
LISTEN_SECONDS = 4.0
MAX_HOSTS = 4096     # refuse to sweep something enormous by accident


def local_subnets() -> list[ipaddress.IPv4Network]:
    """Every IPv4 /n this machine has, minus loopback."""
    out = subprocess.run(["ip", "-4", "-brief", "addr"],
                         capture_output=True, text=True).stdout
    nets = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 3 or parts[0] == "lo":
            continue
        for cidr in parts[2:]:
            try:
                nets.append(ipaddress.ip_network(cidr, strict=False))
            except ValueError:
                pass
    return nets


def sweep(target: str) -> dict[str, bytes]:
    """PING every host in `target` (a CIDR or a single address). Returns replies."""
    try:
        net = ipaddress.ip_network(target, strict=False)
    except ValueError:
        print(f"  not an address or subnet: {target}")
        return {}

    hosts = [net.network_address] if net.num_addresses == 1 else list(net.hosts())
    if len(hosts) > MAX_HOSTS:
        print(f"  {net} has {len(hosts)} hosts; refusing to sweep. "
              f"Narrow it, e.g. a /24.")
        return {}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setblocking(False)

    # Broadcast first. One packet, and it reaches the board whatever address
    # it ended up with -- which is the whole problem this script solves.
    if net.num_addresses > 1:
        try:
            sock.sendto(b"PING", (str(net.broadcast_address), PORT))
        except OSError:
            pass    # some interfaces refuse broadcast; the sweep still runs

    for host in hosts:
        try:
            sock.sendto(b"PING", (str(host), PORT))
        except OSError:
            pass

    print(f"  swept {net} on udp/{PORT}, listening {LISTEN_SECONDS:g}s...")
    found: dict[str, bytes] = {}
    deadline = time.monotonic() + LISTEN_SECONDS
    while time.monotonic() < deadline:
        try:
            data, addr = sock.recvfrom(64)
        except BlockingIOError:
            time.sleep(0.02)
            continue
        except OSError:
            break
        found.setdefault(addr[0], data[:32])
    sock.close()
    return found


def main() -> int:
    targets = sys.argv[1:] or [str(n) for n in local_subnets()]
    if not targets:
        print("no local IPv4 network found; pass a subnet explicitly")
        return 2

    print("looking for the ESP32 (PING -> PONG, udp/%d)\n" % PORT)
    found: dict[str, bytes] = {}
    for t in targets:
        found.update(sweep(t))

    print()
    if found:
        for ip, data in sorted(found.items()):
            reply = data.decode("ascii", errors="replace").strip()
            print(f"  FOUND  {ip}  replied {reply!r}")
        first = sorted(found)[0]
        print("\nUse it:\n"
              f"    ros2 launch mmr_pkg robot.launch.py esp32_ip:={first}\n"
              "\nIf that differs from the address in your launch command, the "
              "stale one was the whole problem.")
        return 0

    print("  NOTHING ANSWERED.\n"
          "  A reply needs all four of: powered, joined to WiFi, listening on\n"
          f"  udp/{PORT}, and on a subnet swept above. Ruling them out, in the\n"
          "  order that costs least:\n\n"
          "  1. Read the boot log over the USB cable. This is the fastest\n"
          "     check by far and it is the one thing the cable is good for:\n"
          "         screen /dev/ttyUSB0 115200      (or /dev/ttyACM0)\n"
          "     then tap the board's EN/RST button. You want:\n"
          "         Ready. IP: <address>\n"
          "         UDP listening on port 1234\n"
          "     'WiFi failed' instead means the firmware is stuck in its\n"
          "     retry loop and will never drive the motors, whatever ROS does.\n"
          "  2. Check the SSID and password compiled into the sketch match the\n"
          "     network that is actually up (esp32/MotionTestOriginal/).\n"
          "  3. Make sure you swept the subnet the ROBOT is on. Run this ON\n"
          "     THE PI if the Pi and this machine are on different networks.\n"
          "  4. Client isolation: some hotspots and guest WiFi block\n"
          "     device-to-device traffic entirely. Nothing on the robot can\n"
          "     fix that; use a network that allows it.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
