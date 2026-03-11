import os
import sys
import time
import struct
import socket
import ctypes
import threading
import subprocess
import tkinter as tk

if sys.platform == "win32":
    hwnd = ctypes.windll.kernel32.GetConsoleWindow()
    if hwnd: ctypes.windll.user32.ShowWindow(hwnd, 0)

PI_SSID = "MyDrone_WiFi"          
PI_IP = "192.168.10.1"            
TX_PORT = 8888
RX_PORT = 8889
FREQ_HZ = 500                     
DEADZONE = 7800

def get_wifi_interface_name():
    try:
        res = subprocess.run("netsh wlan show interfaces", shell=True, capture_output=True, creationflags=0x08000000)
        try: out = res.stdout.decode('gbk')
        except UnicodeDecodeError: out = res.stdout.decode('utf-8', errors='ignore')
        for line in out.split('\n'):
            if "Name" in line or "名称" in line: return line.split(':')[1].strip()
    except: pass
    return "WLAN"

IFACE_NAME = get_wifi_interface_name()

def is_admin():
    try: return ctypes.windll.shell32.IsUserAnAdmin()
    except: return False

if not is_admin():
    script_path = os.path.abspath(sys.argv[0])
    ctypes.windll.shell32.ShellExecuteW(None, "runas", sys.executable, f'"{script_path}"', None, 1)
    sys.exit()

from ctypes import wintypes
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", wintypes.WORD), ("bLeftTrigger", wintypes.BYTE), ("bRightTrigger", wintypes.BYTE),
                ("sThumbLX", wintypes.SHORT), ("sThumbLY", wintypes.SHORT), ("sThumbRX", wintypes.SHORT), ("sThumbRY", wintypes.SHORT)]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wintypes.DWORD), ("Gamepad", XINPUT_GAMEPAD)]

try: xinput = ctypes.windll.xinput1_4
except OSError:
    try: xinput = ctypes.windll.xinput1_3
    except OSError: xinput = None

if xinput:
    xinput.XInputGetState.argtypes = [wintypes.DWORD, ctypes.POINTER(XINPUT_STATE)]
    xinput.XInputGetState.restype = wintypes.DWORD

def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        crc &= 0xFFFF
        for _ in range(8):
            if crc & 0x8000: crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else: crc = (crc << 1) & 0xFFFF
    return crc

class DroneLinkCore:
    def __init__(self):
        self.is_running = False
        self.sock = None
        self.tx_thread = self.rx_thread = None
        self.gamepad_connected = False
        self.ctrl_mode = 0x02
        self.input_vx = self.input_vy = self.input_yaw = self.input_thrust = 0.0
        self.telem_roll = self.telem_pitch = self.telem_yaw = self.telem_pos_z = 0.0
        self.tx_count = self.rx_count = 0
        
        # 🌟 核心：Ping-Pong 高精度时间槽与滑窗
        self.ping_seq = 0                             
        self.sent_times = [0.0] * 256                 
        self.latency_ms = 0.0                         

    def apply_deadzone(self, raw):
        if abs(raw) < DEADZONE: return 0.0
        val = raw - DEADZONE if raw > 0 else raw + DEADZONE
        return max(min(val / (32767.0 - DEADZONE), 1.0), -1.0)

    def start(self):
        if self.is_running: return
        self.is_running = True
        try: ctypes.windll.winmm.timeBeginPeriod(1)
        except: pass
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try: self.sock.ioctl(0x9800000C, False)
        except: pass
        try: self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xC0)
        except: pass
        self.sock.bind(("0.0.0.0", RX_PORT))
        
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start(); self.tx_thread.start()

    def stop(self):
        if not self.is_running: return
        self.is_running = False
        if self.sock:
            # 🌟 终极修复：窗口打烊退出的最后救命急停包，亦采用动态自增序列号，避免被网关误杀
            self.ping_seq = (getattr(self, 'ping_seq', 0) + 1) % 256
            if hasattr(self, 'sent_times'): 
                self.sent_times[self.ping_seq] = time.perf_counter()
            
            payload = struct.pack('<BBBBffff', 0xAA, 0x55, self.ping_seq, 0x02, 0.0, 0.0, 0.0, 0.0)
            packet = payload + struct.pack('<H', calc_crc16(payload[2:]))
            for _ in range(10):
                try: 
                    self.sock.sendto(packet, (PI_IP, TX_PORT))
                    time.sleep(0.005)
                except: pass
            try: self.sock.close()
            except: pass
        try: ctypes.windll.winmm.timeEndPeriod(1)
        except: pass

    def _tx_loop(self):
        target_dt = 1.0 / FREQ_HZ
        next_time = time.perf_counter() + target_dt
        state = XINPUT_STATE()
        
        while self.is_running:
            now = time.perf_counter()
            if next_time - now > 0.0015:
                time.sleep(0.001); continue
            while time.perf_counter() < next_time: pass
            if time.perf_counter() - next_time > 0.01: next_time = time.perf_counter() + target_dt
            else: next_time += target_dt
            
            mode = 0x02; vx = vy = yaw = thrust = 0.0; connected = False

            if xinput and xinput.XInputGetState(0, ctypes.byref(state)) == 0:
                connected = True; mode = 0x01
                thrust = self.apply_deadzone(state.Gamepad.sThumbLY)
                yaw    = self.apply_deadzone(state.Gamepad.sThumbLX)
                vx     = self.apply_deadzone(state.Gamepad.sThumbRY) 
                vy     = self.apply_deadzone(state.Gamepad.sThumbRX)
            
            self.gamepad_connected = connected; self.ctrl_mode = mode
            self.input_vx = vx; self.input_vy = vy; self.input_yaw = yaw; self.input_thrust = thrust
            
            # 🌟 正常发包的动态 Ping 序列自增
            self.ping_seq = (self.ping_seq + 1) % 256
            self.sent_times[self.ping_seq] = time.perf_counter()
            
            payload = struct.pack('<BBBBffff', 0xAA, 0x55, self.ping_seq, mode, vx, vy, thrust, yaw)
            packet = payload + struct.pack('<H', calc_crc16(payload[2:]))
            try:
                self.sock.sendto(packet, (PI_IP, TX_PORT))
                self.tx_count += 1
            except: pass

    def _rx_loop(self):
        while self.is_running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) == 22 and data[0] == 0xBB and data[1] == 0x66:
                    rx_crc, = struct.unpack('<H', data[20:22])
                    if calc_crc16(data[2:20]) == rx_crc:
                        # 🌟 拆包出遥测帧中的 ack_seq (原本是 _pad)
                        msg_id, ack_seq, roll, pitch, yaw_val, pos_z = struct.unpack('<BBffff', data[2:20])
                        if msg_id == 0x10:
                            # 🌟 RTT 物理全环路大闭环测速
                            t_sent = self.sent_times[ack_seq]
                            if t_sent > 0.0:
                                rtt = (time.perf_counter() - t_sent) * 1000.0
                                if 0 < rtt < 1000.0:
                                    if self.latency_ms == 0.0: self.latency_ms = rtt
                                    else: self.latency_ms = self.latency_ms * 0.9 + rtt * 0.1

                            self.telem_roll = roll; self.telem_pitch = pitch
                            self.telem_yaw = yaw_val; self.telem_pos_z = pos_z
                            self.rx_count += 1
            except: time.sleep(0.001)

class DroneGCSApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DroneLink 🚀 全自动地面控制中心 (美国手 Mode 2)")
        self.geometry("720x520")
        self.configure(bg="#1E1E1E")
        self.resizable(False, False)

        self.link = DroneLinkCore()
        self.last_tx = self.last_rx = 0
        self.last_time = time.perf_counter()
        self.is_closing = False
        
        self.init_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        threading.Thread(target=self.auto_connect_workflow, daemon=True).start()
        self.update_telemetry()

    def run_cmd(self, cmd):
        try:
            res = subprocess.run(cmd, shell=True, creationflags=0x08000000, capture_output=True)
            try: return res.stdout.decode('gbk')
            except UnicodeDecodeError: return res.stdout.decode('utf-8', errors='ignore')
        except: return ""

    def tcp_check(self, ip):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((ip, 22))
            return True
        except ConnectionRefusedError: return True 
        except: return False

    def update_status(self, text, color="#00E676"):
        self.after(0, lambda: self.lbl_wizard.config(text=text, fg=color))

    def auto_connect_workflow(self):
        self.after(0, lambda: self.btn_link.config(state="disabled", text="⏳ 正在配置无冲突网络环境..."))
        profiles = self.run_cmd('netsh wlan show profiles')
        if PI_SSID not in profiles:
            self.update_status(f"❌ 找不到配置！请先在系统右下角手动连接一次 {PI_SSID}。", "#FF5252")
            self.after(0, lambda: self.btn_link.config(state="normal", text="🔄 重新尝试连接", bg="#F57C00"))
            return

        self.update_status(f"⚙️ [1/4] 正在净化网卡 ({IFACE_NAME}) 并配置静态 IP...", "#03A9F4")
        self.run_cmd(f'netsh wlan disconnect interface="{IFACE_NAME}"') 
        self.run_cmd('netsh interface ip delete arpcache')              
        self.run_cmd(f'netsh interface ipv4 set address name="{IFACE_NAME}" static 192.168.10.100 255.255.255.0 192.168.10.1')
        time.sleep(1.5)

        self.update_status(f"📡 [2/4] 正在呼叫无人机热点: {PI_SSID} ...", "#03A9F4")
        self.run_cmd(f'netsh wlan connect name="{PI_SSID}" ssid="{PI_SSID}" interface="{IFACE_NAME}"')
        
        connected = False
        for _ in range(15): 
            if self.is_closing: return
            out = self.run_cmd('netsh wlan show interfaces')
            out_lower = out.lower()
            if PI_SSID.lower() in out_lower and ("已连接" in out or "connected" in out_lower):
                connected = True; break
            time.sleep(1)

        if not connected:
            self.update_status(f"❌ 握手超时！可能是树莓派网卡假死，请重启树莓派", "#FF5252")
            self.after(0, lambda: self.btn_link.config(state="normal", text="🔄 重新尝试连接", bg="#F57C00"))
            return

        self.update_status("🔒 [3/4] 握手成功！静态路由已就绪...", "#03A9F4")
        self.update_status("📡 [4/4] 正在验证底层数据链 (探活)...", "#FFC107")
        tcp_ok = False
        for _ in range(10):
            if self.is_closing: return
            if self.tcp_check(PI_IP): tcp_ok = True; break
            time.sleep(1)

        self.run_cmd(f'netsh wlan set autoconfig enabled=no interface="{IFACE_NAME}"')

        if tcp_ok:
            self.update_status("✅ 军工级数据链准备完毕！完全免疫网络抖动！", "#00E676")
            self.after(0, lambda: self.btn_link.config(state="normal", text="▶ 启动 P2P 数据链 (Start)", bg="#2E7D32"))
        else:
            self.update_status("⚠ TCP 探活未响应。可能是防火墙拦截，可尝试强行启动。", "#FFC107")
            self.after(0, lambda: self.btn_link.config(state="normal", text="▶ 强行启动数据链 (Start)", bg="#F57C00"))

    def auto_restore_workflow(self):
        def show_overlay():
            overlay = tk.Frame(self, bg="#111111")
            overlay.place(relx=0, rely=0, relwidth=1, relheight=1)
            tk.Label(overlay, text="🌍 正在安全断开数据链并恢复日常网络，请勿强关...", 
                     fg="#03A9F4", bg="#111111", font=("Microsoft YaHei", 12, "bold")).place(relx=0.5, rely=0.5, anchor="center")
        self.after(0, show_overlay)
        
        self.link.stop() 
        time.sleep(0.5) 
        
        self.run_cmd(f'netsh wlan disconnect interface="{IFACE_NAME}"') 
        self.run_cmd(f'netsh wlan set autoconfig enabled=yes interface="{IFACE_NAME}"')
        self.run_cmd(f'netsh interface ipv4 set address name="{IFACE_NAME}" source=dhcp')
        self.run_cmd(f'netsh interface ipv4 set dnsservers name="{IFACE_NAME}" source=dhcp')
        self.run_cmd('ipconfig /flushdns')
        self.after(0, self.destroy)
        os._exit(0)

    def toggle_link(self):
        if "重新" in self.btn_link["text"]:
            threading.Thread(target=self.auto_connect_workflow, daemon=True).start()
            return
        if self.link.is_running:
            self.link.stop()
            self.btn_link.config(text="▶ 启动 P2P 数据链 (Start)", bg="#2E7D32")
        else:
            self.link.start()
            self.btn_link.config(text="⏹ 断开数据链并急停 (Stop)", bg="#C62828")

    def create_bar(self, parent, label_text):
        frame = tk.Frame(parent, bg="#1E1E1E")
        frame.pack(fill=tk.X, pady=8, padx=5)
        tk.Label(frame, text=label_text, bg="#1E1E1E", fg="#CCCCCC", width=14, anchor="w", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        val_lbl = tk.Label(frame, text=" 0.00", bg="#1E1E1E", fg="#00E676", width=6, font=("Consolas", 11, "bold"))
        val_lbl.pack(side=tk.LEFT)
        canvas = tk.Canvas(frame, width=150, height=18, bg="#333333", highlightthickness=0)
        canvas.pack(side=tk.LEFT, padx=10)
        return val_lbl, canvas

    def init_ui(self):
        f_top = tk.Frame(self, bg="#212121", pady=10)
        f_top.pack(fill=tk.X)
        self.lbl_wizard = tk.Label(f_top, text="程序初始化中...", font=("Microsoft YaHei", 11, "bold"), bg="#212121", fg="#FFC107")
        self.lbl_wizard.pack()

        f_link = tk.Frame(self, bg="#1E1E1E")
        f_link.pack(fill=tk.X, padx=15, pady=15)
        self.btn_link = tk.Button(f_link, text="⏳ 引擎加载中...", font=("Segoe UI", 15, "bold"), bg="#444444", fg="white", height=2, state="disabled", command=self.toggle_link, relief=tk.FLAT, cursor="hand2")
        self.btn_link.pack(fill=tk.X)
        
        f_status = tk.Frame(self, bg="#1E1E1E")
        f_status.pack(fill=tk.X, padx=15, pady=5)
        self.lbl_gamepad = tk.Label(f_status, text="🎮 手柄: 未连接", font=("Microsoft YaHei", 11, "bold"), bg="#1E1E1E", fg="#FF5252")
        self.lbl_gamepad.pack(side=tk.LEFT)
        
        # 🌟 UI 挂载实时延迟指标
        self.lbl_txrx = tk.Label(f_status, text="TX:   0 Hz | RX:   0 Hz | 往返延时: 0.0 ms", font=("Consolas", 11, "bold"), bg="#1E1E1E", fg="#03A9F4")
        self.lbl_txrx.pack(side=tk.RIGHT)

        f_data = tk.LabelFrame(self, text=" 📊 实时控制与遥测面板 (美国手 Mode 2) ", bg="#1E1E1E", fg="#FFC107", font=("Microsoft YaHei", 11, "bold"), padx=10, pady=10)
        f_data.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        
        self.lbl_att = tk.Label(f_data, text="姿态:  等待下行遥测数据...", font=("Consolas", 12, "bold"), bg="#1E1E1E", fg="#FFFFFF", anchor="w")
        self.lbl_att.pack(fill=tk.X, pady=(0, 15))

        f_bars = tk.Frame(f_data, bg="#1E1E1E")
        f_bars.pack(fill=tk.BOTH, expand=True)
        f_left = tk.Frame(f_bars, bg="#1E1E1E")
        f_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        f_right = tk.Frame(f_bars, bg="#1E1E1E")
        f_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.val_thr, self.cvs_thr = self.create_bar(f_left, "↑ 左 油门(THR)")
        self.val_yaw, self.cvs_yaw = self.create_bar(f_left, "↔ 左 偏航(YAW)")
        self.val_pit, self.cvs_pit = self.create_bar(f_right, "↕ 右 俯仰(PIT)")
        self.val_rol, self.cvs_rol = self.create_bar(f_right, "↔ 右 横滚(ROL)")
        
        tk.Label(self, text="💡 提示: 关闭此窗口时，会自动向树莓派发迫降指令并切回日常网络", bg="#1E1E1E", fg="#757575", font=("Microsoft YaHei", 9)).pack(side=tk.BOTTOM, pady=10)

    def update_bar(self, canvas, value):
        canvas.delete("all")
        w = 150; h = 18; center = w // 2
        pos = int((value + 1.0) / 2.0 * w)
        canvas.create_line(center, 0, center, h, fill="#757575", dash=(2, 2))
        if pos > center: canvas.create_rectangle(center, 0, pos, h, fill="#03A9F4", outline="")
        elif pos < center: canvas.create_rectangle(pos, 0, center, h, fill="#03A9F4", outline="")

    def update_telemetry(self):
        now = time.perf_counter()
        dt = now - self.last_time
        if dt >= 1.0:
            tx_rate = int((self.link.tx_count - self.last_tx) / dt)
            rx_rate = int((self.link.rx_count - self.last_rx) / dt)
            self.last_tx, self.last_rx, self.last_time = self.link.tx_count, self.link.rx_count, now
            # 🌟 持续刷新 UI 上的延时显示
            self.lbl_txrx.config(text=f"TX: {tx_rate:3d} Hz | RX: {rx_rate:3d} Hz | 往返延时: {self.link.latency_ms:4.1f} ms")

        if self.link.gamepad_connected:
            m_str = "正常飞行" if self.link.ctrl_mode == 0x01 else "锁死保护"
            self.lbl_gamepad.config(text=f"🎮 手柄: 已连接 [{m_str}]", fg="#00E676")
        else:
            self.lbl_gamepad.config(text="🎮 手柄: 未连接 [失控迫降]", fg="#FF5252")

        self.val_thr.config(text=f"{self.link.input_thrust:5.2f}"); self.update_bar(self.cvs_thr, self.link.input_thrust)
        self.val_yaw.config(text=f"{self.link.input_yaw:5.2f}"); self.update_bar(self.cvs_yaw, self.link.input_yaw)
        self.val_pit.config(text=f"{self.link.input_vx:5.2f}"); self.update_bar(self.cvs_pit, self.link.input_vx)
        self.val_rol.config(text=f"{self.link.input_vy:5.2f}"); self.update_bar(self.cvs_rol, self.link.input_vy)

        r, p, y = self.link.telem_roll * 57.3, self.link.telem_pitch * 57.3, self.link.telem_yaw * 57.3
        self.lbl_att.config(text=f"姿态: R:{r:5.1f}° | P:{p:5.1f}° | Y:{y:5.1f}°  |  高度: {self.link.telem_pos_z:5.2f} m")

        self.after(16, self.update_telemetry)

    def on_closing(self):
        if self.is_closing: return
        self.is_closing = True
        self.protocol("WM_DELETE_WINDOW", lambda: None) 
        threading.Thread(target=self.auto_restore_workflow, daemon=True).start()

if __name__ == "__main__":
    app = DroneGCSApp()
    app.mainloop()