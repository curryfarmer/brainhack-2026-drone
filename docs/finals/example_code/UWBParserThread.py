import threading
import serial
import struct
import time
import serial.tools.list_ports

class UWBParserThread(threading.Thread):
    def __init__(self, x_origin=0.0, y_origin=0.0, serial_port=None, baud_rate=921600):
        super().__init__()
        self.serial_port = serial_port if serial_port else self.detect_com_port()
        self.baud_rate = baud_rate
        self.data_lock = threading.Lock()
        self.tag_data = {}  # Stores tag data {tag_id: (x, y, update_time)}
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.running = True

    def detect_com_port(self):
        """Detects the COM port of the connected USB device."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "USB" in port.description:  # Check for USB device
                print(f"Detected UWB Device on {port.device}")
                return port.device
        print("No UWB device detected.")
        return None

    def run(self):
        """Thread execution function."""
        if not self.serial_port:
            print("No valid COM port detected. Exiting thread.")
            return
        
        try:
            with serial.Serial(self.serial_port, self.baud_rate, timeout=1) as ser:
                buffer = bytearray()
                while self.running:
                    byte = ser.read(1)
                    if not byte:
                        continue

                    if byte == b'U':  # HEADER_BYTE
                        buffer.clear()
                        buffer.append(ord(byte))
                        frame_data = ser.read(895)

                        if len(frame_data) == 895:
                            buffer.extend(frame_data)
                            if buffer[-1] == 0xEE:  # CHECKSUM_BYTE
                                self.parse_data(buffer)
                            else:
                                print("Invalid checksum. Discarding frame.")
                        else:
                            print("Incomplete frame received. Discarding.")
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.running = False

    def parse_data(self, data):
        """Parses the received serial data and updates the tag data."""
        if len(data) < 896:
            print("Insufficient data to parse.")
            return

        frame_header, function_mark = struct.unpack("<BB", data[:2])
        if frame_header != 0x55 or function_mark != 0x00:
            print("Invalid Frame Header or Function Mark")
            return
        self.tag_data = {} 
        offset = 2
        with self.data_lock:
            for _ in range(30):
                if data[offset] != 0xFF:
                    block_id, role = struct.unpack("<BB", data[offset:offset + 2])
                    offset += 2
                    pos_x = int.from_bytes(data[offset:offset + 3], 'little', signed=True) / 1000
                    pos_y = int.from_bytes(data[offset + 3:offset + 6], 'little', signed=True) / 1000
                    offset += 9  # Skip Z position
                    offset += 16  # Skip distance values

                    self.tag_data[block_id] = (pos_x, pos_y, time.time())
                else:
                    offset += 27

    def get_tag_position(self, tag_id):
        """Returns the x, y position and update time of a specific tag."""
        with self.data_lock:
            return self.tag_data.get(tag_id, (None, None, None))

    def stop(self):
        """Stops the thread."""
        self.running = False

# Example Usage
if __name__ == "__main__":
    parser = UWBParserThread()
    if parser.serial_port:
        parser.start()

        try:
            while True:
                tag_id = 0  # Example tag ID
                x, y, update_time = parser.get_tag_position(tag_id)
                if x is not None:
                    print(f"Tag {tag_id}: X={x}, Y={y}, Last updated: {update_time}")
                time.sleep(1)
        except KeyboardInterrupt:
            parser.stop()
            parser.join()
    else:
        print("No UWB device detected. Exiting.")
