#!/usr/bin/env python3

"""
Dola Discovery Listener

Listens on UDP port 8688 for Hula aircraft broadcast packets and
maintains a table of discovered aircraft.
"""

import socket
import threading
import time


class Dola:

    UDP_PORT = 8668

    MAVLINK_STX = 0xFE
    PAYLOAD_LENGTH = 32
    MSG_ID = 232
    CRC_EXTRA = 32

    def __init__(self, listen_ip="0.0.0.0"):
        """
        Create listener.

        listen_ip:
            0.0.0.0 = listen on all interfaces
        """

        self.listen_ip = listen_ip

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow restart without waiting
        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.sock.bind(
            (self.listen_ip, self.UDP_PORT)
        )

        self._running = False
        self._thread = None

        # Thread protection
        self._lock = threading.Lock()
        self._planes = {}

    # ==========================================================
    # Packet Decoder
    # ==========================================================

    def _parse_packet(self, packet, sender_ip):

        # Packet must be exactly 44 bytes
        if len(packet) != 44:
            return None
        if packet[0] != self.MAVLINK_STX:
            return None
        if packet[5] != self.MSG_ID:
            return None
        serial_number = packet[6:22].hex()

        ip_address = (
            packet[22:38]
            .decode("ascii", errors="ignore")
            .rstrip("\x00")
            .strip()
        )

        plane_id = packet[38]
        wifi_mode = packet[39]
        bind_client = packet[40]
        wifi_power = packet[41]

        return {
            "plane_id": plane_id,
            "ip": ip_address,
            "sn": serial_number,
            "wifi_mode": wifi_mode,
            "bind_client": bind_client,
            "wifi_power": wifi_power,
            "sender_ip": sender_ip,
            "last_seen": time.time()
        }

    # ==========================================================
    # Background Listener Thread
    # ==========================================================

    def _listener_loop(self):

        while self._running:

            try:
                packet, addr = self.sock.recvfrom(1024)
                info = self._parse_packet(
                    packet,
                    addr[0]
                )

                if info is None:
                    continue

                plane_id = info["plane_id"]

                with self._lock:

                    # Update latest information
                    self._planes[plane_id] = info

            except OSError:
                break

            except Exception as e:
                print("Parse error:", e)

    # ==========================================================
    # Start / Stop
    # ==========================================================

    def start(self):

        if self._running:
            return

        self._running = True

        self._thread = threading.Thread(
            target=self._listener_loop,
            daemon=True
        )

        self._thread.start()

    def stop(self):

        self._running = False

        try:
            self.sock.close()
        except:
            pass

        if self._thread:
            self._thread.join(timeout=1)

    # ==========================================================
    # User Functions
    # ==========================================================

    def clear(self):
        """
        Clear all cached aircraft.
        """

        with self._lock:
            self._planes.clear()

    def get_ip_by_plane_id(
        self,
        plane_id,
        listen_seconds=5
    ):
        """
        Wait up to listen_seconds for a specific aircraft.

        Returns:
            IP string or None
        """

        deadline = time.time() + listen_seconds

        while time.time() < deadline:

            with self._lock:

                if plane_id in self._planes:
                    return self._planes[plane_id]["ip"]

            time.sleep(0.05)

        return None

    def get_ips_by_plane_ids(
        self,
        plane_ids,
        listen_seconds=5
    ):
        """
        Get IP addresses for multiple aircraft.

        Example return:

        {
            1: "192.168.1.101",
            2: "192.168.1.102",
            3: None
        }
        """

        wanted = set(plane_ids)

        deadline = time.time() + listen_seconds

        while time.time() < deadline:

            with self._lock:

                found_count = sum(
                    1
                    for pid in wanted
                    if pid in self._planes
                )

                if found_count == len(wanted):
                    break

            time.sleep(0.05)

        with self._lock:

            result = {}

            for pid in wanted:

                if pid in self._planes:
                    result[pid] = self._planes[pid]["ip"]
                else:
                    result[pid] = None

            return result

    def get_all_ips(
        self,
        listen_seconds=5
    ):
        """
        Return all discovered aircraft.
        """

        time.sleep(listen_seconds)

        with self._lock:

            return {
                plane_id: info["ip"]
                for plane_id, info
                in self._planes.items()
            }

    def get_all_plane_info(
        self,
        listen_seconds=0
    ):
        """
        Return all cached aircraft information.
        """

        time.sleep(listen_seconds)

        with self._lock:
            return dict(self._planes)


# ==============================================================
# Example Usage
# ==============================================================

def main():

    # Create listener
    dola = Dola()

    # Start background UDP listener
    dola.start()

    try:

        print("Listening on UDP port 8688...")
        print()

        # ------------------------------------------------------
        # Example 1
        # Find a single aircraft
        # ------------------------------------------------------

        print("Searching for Plane ID 1...")

        ip = dola.get_ip_by_plane_id(
            plane_id=1,
            listen_seconds=10
        )

        print("Plane 1 IP =", ip)
        print()

        # ------------------------------------------------------
        # Example 2
        # Find several aircraft
        # ------------------------------------------------------

        print(
            "Searching for Plane IDs "
            "[1, 2, 3, 4, 5]..."
        )

        result = dola.get_ips_by_plane_ids(
            [1, 2, 3, 4, 5],
            listen_seconds=10
        )

        print("Results:")

        for plane_id in sorted(result):

            print(
                f"Plane {plane_id} -> "
                f"{result[plane_id]}"
            )

        print()

        # ------------------------------------------------------
        # Example 3
        # Get all discovered aircraft
        # ------------------------------------------------------

        print(
            "Collecting all aircraft "
            "for 5 seconds..."
        )

        all_ips = dola.get_all_ips(
            listen_seconds=5
        )

        print("Discovered Aircraft:")

        for plane_id, ip in sorted(all_ips.items()):

            print(
                f"Plane {plane_id}: {ip}"
            )

        print()

        # ------------------------------------------------------
        # Example 4
        # Full information
        # ------------------------------------------------------

        print(
            "Detailed aircraft information:"
        )

        info = dola.get_all_plane_info()

        for plane_id in sorted(info):

            aircraft = info[plane_id]

            print()
            print(
                f"Plane ID {plane_id}"
            )
            print(
                f"  IP          : "
                f"{aircraft['ip']}"
            )
            print(
                f"  Serial      : "
                f"{aircraft['sn']}"
            )
            print(
                f"  Wifi Mode   : "
                f"{aircraft['wifi_mode']}"
            )
            print(
                f"  Bind Client : "
                f"{aircraft['bind_client']}"
            )
            print(
                f"  Wifi Power  : "
                f"{aircraft['wifi_power']}"
            )
            print(
                f"  Sender IP   : "
                f"{aircraft['sender_ip']}"
            )

        print()
        print(
            "Listener still running..."
        )
        print(
            "Press Ctrl+C to exit."
        )

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping listener...")

    finally:
        dola.stop()


if __name__ == "__main__":
    main()