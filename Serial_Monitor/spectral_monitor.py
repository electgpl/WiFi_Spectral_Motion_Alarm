import serial
import serial.tools.list_ports
import json
import sys
import time
import threading
from collections import deque

try:
    import msvcrt
except ImportError:
    msvcrt = None


class SpectralMonitor:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.calibrating = True
        self.cal_pct = 0
        self.cal_aps = 0
        self.aps = {}
        self.state = {"aps": 0, "score": 0.0, "alarm": False}
        self.alarm_log = deque(maxlen=20)
        self.score_history = deque(maxlen=60)
        self.lock = threading.Lock()

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            print(f"[OK] Conectado a {self.port} a {self.baud} baud")
            time.sleep(2)
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"[ERROR] No se pudo conectar: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _parse_line(self, line: str):
        line = line.strip()
        if not line or not line.startswith("{"):
            return
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return

        with self.lock:
            t = data.get("t", "")

            if t == "cal":
                self.calibrating = data.get("pct", 0) < 100
                self.cal_pct = data.get("pct", 0)
                self.cal_aps = data.get("aps", 0)

            elif t == "state":
                self.state["aps"] = data.get("aps", 0)
                self.state["score"] = data.get("score", 0.0)
                self.state["alarm"] = data.get("alarm", False)
                self.state["th_high"] = data.get("th_high", 10.0)
                self.state["th_low"] = data.get("th_low", 7.0)
                self.score_history.append(self.state["score"])

            elif t == "ap":
                bssid = data.get("bssid", "")
                self.aps[bssid] = {
                    "rssi": data.get("rssi", 0),
                    "baseline": data.get("baseline", 0.0),
                    "noise": data.get("noise", 0.0),
                    "diff": data.get("diff", 0.0),
                    "misses": data.get("misses", 0),
                    "csi_score": data.get("csi_score", 0.0),
                    "csi_pkt": data.get("csi_pkt", 0),
                }

            elif t == "alarm":
                self.alarm_log.append({
                    "time": time.strftime("%H:%M:%S"),
                    "state": data.get("state", False),
                    "score": data.get("score", 0.0),
                    "reason": data.get("reason", ""),
                })

            elif t == "status":
                self.alarm_log.append({
                    "time": time.strftime("%H:%M:%S"),
                    "state": None,
                    "score": None,
                    "reason": data.get("msg", ""),
                })

    def read_loop(self):
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        self._parse_line(line)
                else:
                    time.sleep(0.01)
            except serial.SerialException:
                print("[ERROR] Perdida de conexion serie")
                self.running = False
                break
            except Exception:
                time.sleep(0.01)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def get_display_data(self):
        with self.lock:
            return {
                "calibrating": self.calibrating,
                "cal_pct": self.cal_pct,
                "cal_aps": self.cal_aps,
                "state": dict(self.state),
                "aps": dict(self.aps),
                "alarm_log": list(self.alarm_log),
                "score_history": list(self.score_history),
            }


def list_ports():
    ports = serial.tools.list_ports.comports()
    esp_ports = []
    for p in sorted(ports):
        desc = f"{p.device} - {p.description}"
        esp_ports.append(p.device)
        if "303A" in p.hwid or "ESP" in p.description.upper() or "CP210" in p.description or "CH340" in p.description:
            print(f"  {desc}  [POSIBLE ESP32]")
        else:
            print(f"  {desc}")
    return esp_ports


def clear_screen():
    print("\033[2J\033[H", end="", flush=True)


def draw_calibration(data):
    pct = data["cal_pct"]
    aps = data["cal_aps"]
    bar_len = 40
    filled = bar_len * pct // 100
    bar = "#" * filled + "." * (bar_len - filled)
    print(f"\033[1;36m{'=' * 80}\033[0m")
    print(f"\033[1;33m{'ESPECTRAL MOTION DETECTOR v3.2-CSI':^80}\033[0m")
    print(f"\033[1;36m{'=' * 80}\033[0m\n")
    print(f"  \033[1;37mCALIBRANDO...\033[0m")
    print(f"  \033[1;37m[{bar}] {pct}%\033[0m")
    print(f"  \033[1;37mRedes encontradas: {aps}\033[0m")
    print(f"\n  \033[1;30mNo se mueva durante la calibracion\033[0m")
    print(f"\n\033[1;36m{'=' * 80}\033[0m")


def draw_monitoring(data):
    state = data["state"]
    aps = data["aps"]
    alarm_log = data["alarm_log"]
    score_hist = data["score_history"]

    clear_screen()

    th_high = state.get("th_high", 10.0)
    th_low = state.get("th_low", 7.0)

    print(f"\033[1;36m{'=' * 80}\033[0m")
    print(f"\033[1;33m{'ESPECTRAL MOTION DETECTOR v3.2-CSI':^80}\033[0m")
    print(f"\033[1;36m{'=' * 80}\033[0m")

    if state["alarm"]:
        print(f"\033[1;41;37m{'  !!ALARMA DISPARADA!!  ':^80}\033[0m")
    else:
        print(f"  \033[1;32mSISTEMA: ACTIVO (monitoreando)\033[0m")

    print(f"  APs:       {state['aps']}")
    print(f"  Score:     \033[1;37m{state['score']:.2f}\033[0m  ", end="")

    if state["score"] > th_high:
        print(f"\033[1;31m>> UMBRAL ({th_high:.1f})\033[0m")
    elif state["score"] > th_low:
        print(f"\033[1;33m~ Umbral bajo ({th_low:.1f})\033[0m")
    else:
        print(f"\033[1;32mNormal\033[0m")

    print(f"\033[1;36m{'-' * 80}\033[0m")
    print(f"  \033[1;37m{'BSSID':<20} {'RSSI':<8} {'Base':<10} {'Ruido':<8} {'Diff':<8} {'Fall':<6} {'CSI':<8}\033[0m")
    print(f"\033[1;36m{'-' * 80}\033[0m")

    sorted_aps = sorted(aps.items(), key=lambda x: x[1].get("noise", 99))
    for bssid, ap in sorted_aps:
        rssi_val = ap["rssi"]
        rssi_str = f"{rssi_val}" if rssi_val > -127 else "---"
        diff = ap["diff"]
        if diff > 2.0:
            color = "\033[1;31m"
        elif diff > 1.0:
            color = "\033[1;33m"
        else:
            color = "\033[1;32m"
        csi = ap.get("csi_score", 0)
        csi_pkt = ap.get("csi_pkt", 0)
        csi_str = f"{csi:.2f}" if csi_pkt > 0 else "---"
        print(f"  {bssid:<20} {rssi_str:<8} {ap['baseline']:<10.1f} {ap['noise']:<8.1f} "
              f"{color}{diff:<8.2f}\033[0m {ap['misses']:<6} {csi_str:<8}")

    print(f"\033[1;36m{'-' * 80}\033[0m")

    if score_hist:
        max_s = max(max(score_hist), 0.1)
        print(f"  \033[1;37mUltimos scores:\033[0m  ", end="")
        hist = score_hist[-25:]
        for s in hist:
            if s > th_high:
                c = "\033[1;31m"
            elif s > th_low:
                c = "\033[1;33m"
            else:
                c = "\033[1;32m"
            h = max(1, int((s / max_s) * 3))
            print(f"{c}{'#' * h}\033[0m", end="")
        print()

    if alarm_log:
        last = alarm_log[-3:]
        print(f"\033[1;36m{'-' * 80}\033[0m")
        print(f"  \033[1;37mEventos:\033[0m")
        for entry in last:
            st = entry["state"]
            if st is True:
                st_str = "\033[1;41;37m ALARMA \033[0m"
            elif st is False:
                st_str = "\033[1;42;37m NORMAL \033[0m"
            else:
                st_str = "\033[1;44;37m INFO \033[0m"
            sc = f" score={entry['score']:.1f}" if entry["score"] is not None else ""
            print(f"  {st_str}{sc}  {entry['reason']}")

    print(f"\n\033[1;36m{'=' * 80}\033[0m")
    print(f"  \033[1;30mPresione 'q' para salir | Ctrl+C para interrumpir\033[0m")
    print(f"\033[1;36m{'=' * 80}\033[0m")


def draw_interface(monitor: SpectralMonitor):
    data = monitor.get_display_data()

    if data["calibrating"] and data["cal_pct"] < 100:
        draw_calibration(data)
    else:
        draw_monitoring(data)


def main():
    port = None
    baud = 115200

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            baud = int(sys.argv[2])
        except ValueError:
            pass

    if not port:
        print("Puertos serie disponibles:")
        available = list_ports()
        print()
        if not available:
            print("[ERROR] No se encontraron puertos serie.")
            print("  Uso: python spectral_monitor.py <PUERTO> [BAUD]")
            print("  Ej:  python spectral_monitor.py COM4")
            sys.exit(1)
        port = available[0]
        print(f"Usando primer puerto disponible: {port}")
        print(f"  (especifique el puerto como argumento para cambiarlo)")
        print()

    monitor = SpectralMonitor(port, baud)
    if not monitor.connect():
        sys.exit(1)

    monitor.start()

    try:
        while monitor.running:
            draw_interface(monitor)
            if msvcrt and msvcrt.kbhit():
                key = msvcrt.getch()
                if key.lower() == b'q':
                    break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()
        monitor.disconnect()
        print("\nMonitor detenido.")


if __name__ == "__main__":
    main()
