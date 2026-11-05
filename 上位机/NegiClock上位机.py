# -*- coding: utf-8 -*-
"""
NegiClock 上位机（校时工具）
=============================

功能
----
1. 选择串口 + 波特率（默认 9600）后“绑定”，绑定成功会立即校时一次；
2. 之后每 5 分钟自动给单片机校时一次，只有发送时才占用串口，发完立刻释放；
3. 单片机收到后回执 ACK，收不到就延迟重发，最多尝试 3 次；
   3 次都失败和成功一样，继续下一次 5 分钟倒计时；
4. 关闭窗口后程序隐藏到托盘，托盘菜单可重新打开界面 / 立即同步 / 退出；
5. 界面深色主题。

协议（与单片机 main.c / Serial.c 对应）
----------------------------------------
上位机 -> 单片机：
    A5 5A LEN CMD DATA... XOR            XOR = 前面所有字节异或
    CMD=0x00  心跳（用于握手 / 探测波特率）
    CMD=0x01  校时，DATA = YY MM DD W HH MI SS（二进制）
单片机 -> 上位机：
    A5 5A 01 06 XOR

关于波特率
----------
12MHz 晶振下 8051 用 SMOD=1、TH1=0xFA 时实际是 10417bps；11.0592MHz 时正好 9600。
所以默认先用 9600，勾选“自动匹配”时如果握手不通会自动改试 10417 / 4808。
"""

import json
import os
import queue
import sys
import threading
import time
import traceback
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except Exception:                                    # pragma: no cover
    serial = None
    list_ports = None

try:
    from PIL import Image, ImageDraw
    import pystray
except Exception:                                    # pragma: no cover
    pystray = None
    Image = None

# ============================== 常量 ==============================
APP_TITLE = "NegiClock 桌面时钟 · 校时工具"
APP_NAME = "NegiClock"

CONF_DIR = os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), APP_NAME)
CONF_PATH = os.path.join(CONF_DIR, "config.json")
LOG_PATH = os.path.join(CONF_DIR, "negiclock.log")

SYNC_INTERVAL = 5 * 60          # 自动校时间隔（秒）
ACK_TIMEOUT = 1.2               # 等回执时间（秒）
MAX_TRY = 3                     # 最多尝试次数

BAUD_CHOICES = ["9600", "10417", "4808"]
BAUD_AUTO_ORDER = [9600, 10417, 4808]

# 深色主题配色
BG        = "#1e1f22"
BG_PANEL  = "#26282c"
FIELD     = "#303338"
FG        = "#e6e6e6"
FG_DIM    = "#9aa0a6"
ACCENT    = "#4ec9b0"
ACCENT_2  = "#569cd6"
DANGER    = "#e06c75"
BORDER    = "#3a3d42"

ACK_BYTES = bytes([0xA5, 0x5A, 0x01, 0x06, 0xA5 ^ 0x5A ^ 0x01 ^ 0x06])


# ============================== 协议 ==============================
def _frame(body):
    chk = 0
    for b in body:
        chk ^= b
    return bytes(body + [chk])


def frame_ping():
    return _frame([0xA5, 0x5A, 0x01, 0x00])


def frame_set(dt):
    data = [dt.year % 100, dt.month, dt.day, dt.isoweekday(),
            dt.hour, dt.minute, dt.second]
    return _frame([0xA5, 0x5A, 0x08, 0x01] + data)


def frame_read_alarms():
    """请单片机把两个闹钟发回来。"""
    return _frame([0xA5, 0x5A, 0x01, 0x02])


def frame_set_alarms(alarms):
    """alarms = [(hour, minute, mode), (hour, minute, mode)]"""
    body = [0xA5, 0x5A, 0x07, 0x03]
    for (h, m, md) in alarms:
        body += [int(h) & 0xFF, int(m) & 0xFF, int(md) & 0xFF]
    return _frame(body)


# 闹钟响铃方式，下标必须和固件里的 AL_* 一致
ALARM_MODES = ["关闭", "工作日", "每天", "一次"]


# ============================== 串口 ==============================
def open_port(port, baud):
    """打开串口。

    重点：DTR / RTS 必须在 open() **之前** 就设成无效。
    板上的“USB自动下载电路”把 CH340 的 RTS#(14) / DTR#(13) 经两个 2TY
    三极管接到了单片机的 RST 上；而 pyserial 的 _dtr_state / _rts_state
    默认是 True，open() 时会把这两个脚拉成有效电平 —— 于是单片机被复位
    （表现为“滴、滴”两声重新开机）。pyserial 的 dtr/rts 属性在端口没打开
    时只是记住状态，open() 时用它去配置 DCB，所以要在 open() 前赋值。
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.05
    ser.write_timeout = 1.5

    # 先设定电平状态，再打开，open() 时直接以“无效电平”配置 DCB
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    ser.open()

    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    return ser


def wait_ack(ser, timeout=ACK_TIMEOUT):
    """在 timeout 内找 5 字节回执。"""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(1)
        if chunk:
            buf += chunk
            if len(buf) > 64:
                del buf[:-64]
            if ACK_BYTES in bytes(buf):
                return True
    return False


def wait_frame(ser, want_cmd, timeout=1.5):
    """在 timeout 内等一帧 A5 5A LEN CMD ... XOR，返回 payload(list) 或 None。"""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(1)
        if not chunk:
            continue
        buf += chunk
        if len(buf) > 96:
            del buf[:-96]

        i = 0
        while i < len(buf):
            if buf[i] != 0xA5:
                i += 1
                continue
            if i + 3 >= len(buf):
                break
            if buf[i + 1] != 0x5A:
                i += 1
                continue
            ln = buf[i + 2]
            if ln < 1 or ln > 12:
                i += 1
                continue
            end = i + 3 + ln + 1          # header(3) + payload + checksum
            if end > len(buf):
                break                     # wait for more bytes
            x = 0
            for b in buf[i:i + 3 + ln]:
                x ^= b
            if x == buf[end - 1] and buf[i + 3] == want_cmd:
                return list(buf[i + 3:end - 1])
            i += 1
    return None


# ============================== 主程序 ==============================
class App(object):
    def __init__(self, root):
        self.root = root
        self.q = queue.Queue()               # 工作线程 -> UI 线程
        self.icon = None
        self.tray_ok = False

        self.port = None                     # 已绑定的串口名
        self.baud = 9600                     # 实际使用的波特率
        self.busy = False                    # 正在同步
        self.alarm_busy = False              # 正在读写闹钟
        self.next_sync = None                # 下次自动校时的时间戳
        self.last_result = ""

        self._build_style()
        self._build_ui()
        self._load_config()
        self._refresh_ports()

        self._start_tray()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._apply_dark_titlebar()
        self.root.after(100, self._drain)
        self.root.after(1000, self._tick)

        self.log("程序已启动。请选择串口后点击“绑定串口”。")
        if serial is None:
            self.log("!! 没有找到 pyserial，请先安装：pip install pyserial", "err")

    # ------------------------------ 界面 ------------------------------
    def _build_style(self):
        st = ttk.Style(self.root)
        try:
            st.theme_use("clam")
        except Exception:
            pass

        self.root.configure(bg=BG)
        st.configure(".", background=BG, foreground=FG, fieldbackground=FIELD,
                     bordercolor=BORDER, lightcolor=BG_PANEL, darkcolor=BG_PANEL,
                     font=("Microsoft YaHei UI", 9))
        st.configure("TFrame", background=BG)
        st.configure("Panel.TFrame", background=BG_PANEL)
        st.configure("TLabel", background=BG, foreground=FG)
        st.configure("Dim.TLabel", background=BG, foreground=FG_DIM)
        st.configure("Title.TLabel", background=BG, foreground=ACCENT,
                     font=("Microsoft YaHei UI", 13, "bold"))
        st.configure("Status.TLabel", background=BG_PANEL, foreground=FG_DIM)
        st.configure("TButton", background=FIELD, foreground=FG, borderwidth=0,
                     focusthickness=0, padding=(10, 5))
        st.map("TButton",
               background=[("active", "#3d4147"), ("disabled", "#2a2c30")],
               foreground=[("disabled", "#6b7075")])
        st.configure("Accent.TButton", background=ACCENT_2, foreground="#ffffff")
        st.map("Accent.TButton", background=[("active", "#6aaee8")])
        st.configure("TCombobox", fieldbackground=FIELD, background=FIELD,
                     foreground=FG, arrowcolor=FG, bordercolor=BORDER)
        st.map("TCombobox",
               fieldbackground=[("readonly", FIELD)],
               foreground=[("readonly", FG)],
               selectbackground=[("readonly", FIELD)],
               selectforeground=[("readonly", FG)])
        st.configure("TCheckbutton", background=BG, foreground=FG)
        st.map("TCheckbutton", background=[("active", BG)])
        st.configure("TSeparator", background=BORDER)
        st.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", background=BG_PANEL, foreground=FG_DIM,
                     padding=(16, 6), borderwidth=0)
        st.map("TNotebook.Tab",
               background=[("selected", FIELD)],
               foreground=[("selected", FG)])
        st.configure("TSpinbox", fieldbackground=FIELD, foreground=FG,
                     background=FIELD, arrowcolor=FG, bordercolor=BORDER,
                     lightcolor=BG_PANEL, darkcolor=BG_PANEL)

        self.root.option_add("*TCombobox*Listbox.background", FIELD)
        self.root.option_add("*TCombobox*Listbox.foreground", FG)
        self.root.option_add("*TCombobox*Listbox.selectBackground", ACCENT_2)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

    def _build_ui(self):
        root = self.root
        root.title(APP_TITLE)
        root.geometry("660x600")
        root.minsize(600, 560)

        pad = {"padx": 14, "pady": 6}

        # ---- 标题 ----
        head = ttk.Frame(root)
        head.pack(fill="x", padx=14, pady=(12, 2))
        ttk.Label(head, text="NegiClock", style="Title.TLabel").pack(side="left")
        ttk.Label(head, text="  桌面时钟 · 自动校时守护", style="Dim.TLabel").pack(side="left")

        ttk.Separator(root, orient="horizontal").pack(fill="x", padx=14, pady=(8, 4))

        # ---- 串口设置 ----
        box = ttk.Frame(root)
        box.pack(fill="x", **pad)

        row1 = ttk.Frame(box)
        row1.pack(fill="x")
        ttk.Label(row1, text="串口").pack(side="left")
        self.cb_port = ttk.Combobox(row1, width=12, state="readonly", values=[])
        self.cb_port.pack(side="left", padx=(6, 6))
        self.btn_refresh = ttk.Button(row1, text="刷新", command=self._refresh_ports)
        self.btn_refresh.pack(side="left")

        ttk.Label(row1, text="  波特率").pack(side="left")
        self.cb_baud = ttk.Combobox(row1, width=8, state="readonly", values=BAUD_CHOICES)
        self.cb_baud.set("9600")
        self.cb_baud.pack(side="left", padx=(6, 8))
        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(row1, text="自动匹配", variable=self.var_auto).pack(side="left")

        row2 = ttk.Frame(box)
        row2.pack(fill="x", pady=(10, 0))
        self.btn_bind = ttk.Button(row2, text="绑定串口", style="Accent.TButton",
                                   command=self.on_bind)
        self.btn_bind.pack(side="left")
        self.btn_sync = ttk.Button(row2, text="立即同步", command=self.on_manual_sync,
                                   state="disabled")
        self.btn_sync.pack(side="left", padx=(8, 0))
        self.btn_hide = ttk.Button(row2, text="隐藏到托盘", command=self.hide_window)
        self.btn_hide.pack(side="left", padx=(8, 0))

        # ---- 功能页 ----
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=False, padx=14, pady=(10, 4))
        self.nb = nb

        # ===== 页 1: 校时 =====
        tab_time = ttk.Frame(nb)
        nb.add(tab_time, text="   校时   ")

        panel = ttk.Frame(tab_time, style="Panel.TFrame", relief="flat")
        panel.pack(fill="x", padx=2, pady=(10, 6))
        self.lbl_state = ttk.Label(panel, text="状态：未绑定", style="Status.TLabel")
        self.lbl_state.pack(anchor="w", padx=12, pady=(8, 2))
        self.lbl_next = ttk.Label(panel, text="下次自动同步：--", style="Status.TLabel")
        self.lbl_next.pack(anchor="w", padx=12, pady=(0, 8))

        ttk.Label(tab_time,
                  text="绑定后每 5 分钟自动校时一次；只有发送时才占用串口。\n"
                       "关闭窗口会隐藏到托盘，双击托盘图标可以再打开。",
                  style="Dim.TLabel", justify="left").pack(anchor="w", padx=6, pady=(2, 8))

        # ===== 页 2: 闹钟 =====
        tab_al = ttk.Frame(nb)
        nb.add(tab_al, text="   闹钟   ")

        hint = ttk.Label(tab_al,
                         text="两个闹钟分别对应开发板的 A1 / A2 页面。\n"
                              "点“从设备同步”把板子里的设置读上来，改好之后点“写入设备”。",
                         style="Dim.TLabel", justify="left")
        hint.pack(anchor="w", padx=8, pady=(10, 4))

        self.al_vars = []
        for i in (0, 1):
            box = ttk.Frame(tab_al)
            box.pack(fill="x", padx=8, pady=6)

            ttk.Label(box, text="闹钟 %d" % (i + 1),
                      style="Title.TLabel").pack(side="left")

            ttk.Label(box, text="    时").pack(side="left")
            sv_h = tk.StringVar(value="07" if i == 0 else "22")
            sp_h = ttk.Spinbox(box, from_=0, to=23, width=4, textvariable=sv_h,
                               increment=1, wrap=True)
            sp_h.pack(side="left", padx=4)

            ttk.Label(box, text="分").pack(side="left")
            sv_m = tk.StringVar(value="30" if i == 0 else "00")
            sp_m = ttk.Spinbox(box, from_=0, to=59, width=4, textvariable=sv_m,
                               increment=1, wrap=True)
            sp_m.pack(side="left", padx=4)

            ttk.Label(box, text="  响铃").pack(side="left")
            sv_md = tk.StringVar(value=ALARM_MODES[0])
            ttk.Combobox(box, width=8, state="readonly", textvariable=sv_md,
                         values=ALARM_MODES).pack(side="left", padx=4)

            self.al_vars.append((sv_h, sv_m, sv_md))

        alrow = ttk.Frame(tab_al)
        alrow.pack(fill="x", padx=8, pady=(8, 4))
        self.btn_al_read = ttk.Button(alrow, text="从设备同步",
                                      command=self.on_alarm_read, state="disabled")
        self.btn_al_read.pack(side="left")
        self.btn_al_write = ttk.Button(alrow, text="写入设备", style="Accent.TButton",
                                       command=self.on_alarm_write, state="disabled")
        self.btn_al_write.pack(side="left", padx=(8, 0))

        self.lbl_al = ttk.Label(tab_al, text="状态：未绑定", style="Dim.TLabel")
        self.lbl_al.pack(anchor="w", padx=8, pady=(4, 8))

        # ---- 日志 ----
        logrow = ttk.Frame(root)
        logrow.pack(fill="both", expand=True, padx=14, pady=(6, 4))
        self.txt = tk.Text(logrow, bg="#17181a", fg=FG, insertbackground=FG,
                           relief="flat", wrap="word", height=8,
                           font=("Consolas", 9), padx=8, pady=6)
        sb = ttk.Scrollbar(logrow, orient="vertical", command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.txt.tag_config("ok", foreground=ACCENT)
        self.txt.tag_config("err", foreground=DANGER)
        self.txt.tag_config("dim", foreground=FG_DIM)
        self.txt.configure(state="disabled")

        # ---- 底部按钮 ----
        foot = ttk.Frame(root)
        foot.pack(fill="x", padx=14, pady=(0, 10))
        ttk.Button(foot, text="退出程序", command=self.quit_app).pack(side="right")
        ttk.Label(foot, text="关闭窗口即隐藏到托盘", style="Dim.TLabel").pack(side="left")

    # ------------------------------ 窗口外观 ------------------------------
    def _apply_dark_titlebar(self):
        """Windows 10/11：把系统标题栏也画成深色，和界面统一。"""
        try:
            import ctypes
            self.root.update_idletasks()
            hwnd = ctypes.windll.user32.GetParent(self.root.winfo_id())
            flag = ctypes.c_int(1)
            for attr in (20, 19):        # DWMWA_USE_IMMERSIVE_DARK_MODE（旧版是 19）
                ctypes.windll.dwmapi.DwmSetWindowAttribute(
                    hwnd, attr, ctypes.byref(flag), ctypes.sizeof(flag))
        except Exception:
            pass

    # ------------------------------ 日志 ------------------------------
    def log(self, msg, tag=None):
        ts = datetime.now().strftime("%H:%M:%S")
        line = "[%s] %s" % (ts, msg)
        self.txt.configure(state="normal")
        self.txt.insert("end", line + "\n", tag or ())
        self.txt.see("end")
        self.txt.configure(state="disabled")
        try:
            os.makedirs(CONF_DIR, exist_ok=True)
            with open(LOG_PATH, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass

    def _post(self, fn, *a):
        self.q.put((fn, a))

    def _drain(self):
        try:
            while True:
                fn, a = self.q.get_nowait()
                try:
                    fn(*a)
                except Exception:
                    self.log("内部错误：" + traceback.format_exc(limit=2), "err")
        except queue.Empty:
            pass
        self.root.after(100, self._drain)

    # ------------------------------ 配置 ------------------------------
    def _load_config(self):
        try:
            with open(CONF_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if cfg.get("port"):
                self.cb_port.set(cfg["port"])
            if cfg.get("baud"):
                self.cb_baud.set(str(cfg["baud"]))
            self.var_auto.set(bool(cfg.get("auto", True)))
        except Exception:
            pass

    def _save_config(self):
        try:
            os.makedirs(CONF_DIR, exist_ok=True)
            with open(CONF_PATH, "w", encoding="utf-8") as f:
                json.dump({"port": self.port or "",
                           "baud": int(self.cb_baud.get() or 9600),
                           "auto": bool(self.var_auto.get())}, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    # ------------------------------ 串口 ------------------------------
    def _refresh_ports(self):
        if list_ports is None:
            return
        ports = []
        try:
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            pass
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.set(ports[0])

    def on_bind(self):
        if serial is None:
            messagebox.showerror("缺少依赖", "没有找到 pyserial，请先执行：\npip install pyserial")
            return
        port = self.cb_port.get().strip()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return

        try:
            baud = int(self.cb_baud.get())
        except Exception:
            baud = 9600

        self.btn_bind.configure(state="disabled", text="绑定中…")
        self.log("正在与 %s 握手…" % port)
        threading.Thread(target=self._bind_worker, args=(port, baud), daemon=True).start()

    def _bind_worker(self, port, baud):
        if self.var_auto.get():
            # 用户选择的波特率优先，再自动试其它可能值
            cands = [baud] + [b for b in BAUD_AUTO_ORDER if b != baud]
        else:
            cands = [baud]
        for b in cands:
            ok = False
            try:
                ser = open_port(port, b)
                try:
                    time.sleep(0.35)
                    ser.reset_input_buffer()
                    ser.write(frame_ping())
                    ser.flush()
                    ok = wait_ack(ser, 1.0)
                finally:
                    ser.close()
            except Exception as e:
                self._post(self._bind_done, port, b, False, str(e))
                return
            if ok:
                self._post(self._bind_done, port, b, True, "")
                return
        self._post(self._bind_done, port, baud, False, "单片机没有回执")

    def _bind_done(self, port, baud, ok, err):
        self.btn_bind.configure(state="normal", text="绑定串口")
        if not ok:
            self.port = None
            self.lbl_state.configure(text="状态：绑定失败（%s）" % err)
            self.log("绑定失败：%s。请检查串口是否选对、开发板是否上电。" % err, "err")
            self.btn_sync.configure(state="disabled")
            self.btn_al_read.configure(state="disabled")
            self.btn_al_write.configure(state="disabled")
            self.lbl_al.configure(text="状态：未绑定")
            return

        self.port = port
        self.baud = baud
        self.cb_baud.set(str(baud))
        self._save_config()
        self.lbl_state.configure(text="状态：已绑定 %s @ %d" % (port, baud))
        self.log("绑定成功：%s @ %d（已关闭串口，需要发送时才占用）" % (port, baud), "ok")
        self.btn_sync.configure(state="normal")
        self.btn_al_read.configure(state="normal")
        self.btn_al_write.configure(state="normal")
        self.lbl_al.configure(text="状态：已绑定 %s @ %d" % (port, baud))
        self._start_sync("绑定后立即校时")

    # ------------------------------ 闹钟 ------------------------------
    def _alarm_values(self):
        """从界面读出 [(h, m, mode), (h, m, mode)]"""
        out = []
        for (sv_h, sv_m, sv_md) in self.al_vars:
            try:
                h = int(sv_h.get()) % 24
            except Exception:
                h = 0
            try:
                m = int(sv_m.get()) % 60
            except Exception:
                m = 0
            try:
                md = ALARM_MODES.index(sv_md.get())
            except Exception:
                md = 0
            out.append((h, m, md))
        return out

    def _alarm_set_values(self, alarms):
        for (sv_h, sv_m, sv_md), (h, m, md) in zip(self.al_vars, alarms):
            sv_h.set("%02d" % (h % 24))
            sv_m.set("%02d" % (m % 60))
            sv_md.set(ALARM_MODES[md] if 0 <= md < len(ALARM_MODES) else ALARM_MODES[0])

    def on_alarm_read(self):
        if not self.port:
            messagebox.showwarning("提示", "请先绑定串口")
            return
        if self.alarm_busy:
            self.log("闹钟操作进行中，请稍候…", "dim")
            return
        self.alarm_busy = True
        self.btn_al_read.configure(state="disabled")
        self.btn_al_write.configure(state="disabled")
        self.lbl_al.configure(text="状态：正在从设备读取…")
        self.log("向单片机读取闹钟设置…")
        threading.Thread(target=self._alarm_worker, args=("read", None), daemon=True).start()

    def on_alarm_write(self):
        if not self.port:
            messagebox.showwarning("提示", "请先绑定串口")
            return
        if self.alarm_busy:
            self.log("闹钟操作进行中，请稍候…", "dim")
            return
        alarms = self._alarm_values()
        self.alarm_busy = True
        self.btn_al_read.configure(state="disabled")
        self.btn_al_write.configure(state="disabled")
        self.lbl_al.configure(text="状态：正在写入…")
        self.log("写入闹钟：A1 %02d:%02d %s / A2 %02d:%02d %s"
                 % (alarms[0][0], alarms[0][1], ALARM_MODES[alarms[0][2]],
                    alarms[1][0], alarms[1][1], ALARM_MODES[alarms[1][2]]))
        threading.Thread(target=self._alarm_worker, args=("write", alarms),
                         daemon=True).start()

    def _alarm_worker(self, what, alarms):
        port, baud = self.port, self.baud
        ok = False
        resp = None
        err = ""
        try:
            ser = open_port(port, baud)
            try:
                time.sleep(0.35)
                ser.reset_input_buffer()
                if what == "write":
                    ser.write(frame_set_alarms(alarms))
                    ser.flush()
                    ok = wait_ack(ser, 1.5)
                    if not ok:
                        err = "没有收到回执"
                else:
                    ser.write(frame_read_alarms())
                    ser.flush()
                    resp = wait_frame(ser, 0x82, 1.5)
                    ok = resp is not None
                    if not ok:
                        err = "没有收到闹钟数据"
            finally:
                ser.close()
        except Exception as e:
            err = str(e)

        self._post(self._alarm_done, what, ok, resp, err)

    def _alarm_done(self, what, ok, resp, err):
        self.alarm_busy = False
        self.btn_al_read.configure(state="normal" if self.port else "disabled")
        self.btn_al_write.configure(state="normal" if self.port else "disabled")

        if what == "write":
            if ok:
                self.lbl_al.configure(text="状态：写入成功（开发板会“滴”一声）")
                self.log("闹钟写入成功，收到回执。", "ok")
            else:
                self.lbl_al.configure(text="状态：写入失败（%s）" % err)
                self.log("闹钟写入失败：%s" % err, "err")
            return

        if ok and resp and len(resp) >= 7:
            alarms = [(resp[1], resp[2], resp[3]), (resp[4], resp[5], resp[6])]
            self._alarm_set_values(alarms)
            self.lbl_al.configure(text="状态：已从设备读取")
            self.log("读到闹钟：A1 %02d:%02d %s / A2 %02d:%02d %s"
                     % (alarms[0][0], alarms[0][1], ALARM_MODES[alarms[0][2]],
                        alarms[1][0], alarms[1][1], ALARM_MODES[alarms[1][2]]), "ok")
        else:
            self.lbl_al.configure(text="状态：读取失败（%s）" % err)
            self.log("闹钟读取失败：%s" % err, "err")

    def _ensure_tray(self):
        """隐藏到托盘前保证托盘图标存在。"""
        return self.tray_ok

    def hide_window(self):
        if not self._ensure_tray():
            messagebox.showinfo("提示", "托盘图标不可用，窗口将最小化。")
            self.root.iconify()
            return
        self.root.withdraw()
        try:
            self.icon.notify("NegiClock 仍在后台运行，每 5 分钟自动校时。", "NegiClock")
        except Exception:
            pass

    def show_window(self):
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()

    def on_close(self):
        if not self.port:
            self.quit_app()              # 还没绑定，直接退出
        elif self.tray_ok:
            self.hide_window()           # 绑定后关闭窗口 = 隐藏到托盘
        else:
            # 托盘不可用时不能真的藏起来，否则找不回来
            self.root.iconify()
            self.log("托盘图标不可用，已最小化到任务栏（不会退出，仍在后台定时校时）。", "dim")

    def quit_app(self):
        try:
            if self.icon:
                self.icon.stop()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    # ------------------------------ 托盘 ------------------------------
    def _tray_image(self):
        img = Image.new("RGBA", (64, 64), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        d.ellipse((3, 3, 60, 60), fill="#1e1f22", outline="#4ec9b0", width=4)
        d.line((32, 32, 32, 15), fill="#e6e6e6", width=4)      # 时针
        d.line((32, 32, 46, 39), fill="#e6e6e6", width=3)      # 分针
        d.ellipse((29, 29, 35, 35), fill="#4ec9b0")
        return img

    def _start_tray(self):
        if pystray is None or Image is None:
            self.tray_ok = False
            return
        try:
            menu = pystray.Menu(
                pystray.MenuItem("打开主界面", lambda *a: self._post(self.show_window), default=True),
                pystray.MenuItem("立即同步", lambda *a: self._post(self.on_manual_sync)),
                pystray.Menu.SEPARATOR,
                pystray.MenuItem("退出", lambda *a: self._post(self.quit_app)),
            )
            self.icon = pystray.Icon("NegiClock", self._tray_image(),
                                     "NegiClock 桌面时钟", menu)
            ready = threading.Event()
            state = {"ok": False}

            def _setup(icon):
                try:
                    icon.visible = True
                except Exception:
                    pass
                state["ok"] = True
                ready.set()

            def _run():
                try:
                    self.icon.run(setup=_setup)
                except Exception:
                    pass
                finally:
                    ready.set()          # 失败也要放行，避免主线程死等

            threading.Thread(target=_run, daemon=True).start()
            ready.wait(2.0)
            self.tray_ok = bool(state["ok"])   # 只有图标真的建起来才算可用
        except Exception:
            self.tray_ok = False

    # ------------------------------ 同步 ------------------------------
    def on_manual_sync(self):
        if not self.port:
            messagebox.showwarning("提示", "请先绑定串口")
            return
        if self.busy:
            self.log("正在同步中，请稍候…", "dim")
            return
        self._start_sync("手动同步")

    def _start_sync(self, reason):
        if self.busy or not self.port:
            return
        self.busy = True
        self.btn_sync.configure(state="disabled")
        self.lbl_next.configure(text="下次自动同步：同步中…")
        threading.Thread(target=self._sync_worker, args=(reason,), daemon=True).start()

    def _sync_worker(self, reason):
        port, baud = self.port, self.baud
        now = datetime.now()
        if reason != "自动同步":
            self._post(self.log, "开始同步（%s）：%s" % (reason, now.strftime("%Y-%m-%d %H:%M:%S")))

        ok = False
        for attempt in range(1, MAX_TRY + 1):
            frame = frame_set(datetime.now())
            try:
                ser = open_port(port, baud)
            except Exception as e:
                self._post(self.log, "第 %d 次：打开 %s 失败（%s）" % (attempt, port, e), "err")
                time.sleep(0.4)
                continue
            try:
                # 打开后先等一会儿：万一复位还是发生了（比如驱动在 open 瞬间
                # 抖动了一下 DTR），单片机要 300ms 左右才跑完初始化并准备好。
                time.sleep(0.35)
                ser.reset_input_buffer()
                ser.write(frame)
                ser.flush()
                ok = wait_ack(ser, ACK_TIMEOUT)
            except Exception as e:
                self._post(self.log, "第 %d 次：发送异常（%s）" % (attempt, e), "err")
            finally:
                try:
                    ser.close()                 # 发完立即释放串口
                except Exception:
                    pass

            if ok:
                self._post(self._sync_done, True, attempt, "")
                return
            if attempt < MAX_TRY:
                self._post(self.log, "第 %d 次没有回执，1 秒后重发…" % attempt, "dim")
                time.sleep(0.8)

        self._post(self._sync_done, False, MAX_TRY, "3 次都没有收到回执")

    def _sync_done(self, ok, tries, err):
        self.busy = False
        self.btn_sync.configure(state="normal")
        self.last_result = "成功" if ok else "失败"
        ts = datetime.now().strftime("%H:%M:%S")
        if ok:
            self.log("同步成功（第 %d 次尝试，%s）" % (tries, ts), "ok")
        else:
            self.log("同步失败：%s（%s）。继续下一次 5 分钟倒计时。" % (err, ts), "err")
        self.next_sync = time.time() + SYNC_INTERVAL
        self._update_next_label()

    def _tick(self):
        if self.port and self.next_sync and not self.busy and time.time() >= self.next_sync:
            self._start_sync("自动同步")
        self._update_next_label()
        self.root.after(500, self._tick)

    def _update_next_label(self):
        if not self.port:
            self.lbl_next.configure(text="下次自动同步：--")
            return
        if self.busy:
            self.lbl_next.configure(text="下次自动同步：同步中…")
            return
        if not self.next_sync:
            self.lbl_next.configure(text="下次自动同步：--")
            return
        left = max(0, int(self.next_sync - time.time()))
        self.lbl_next.configure(text="下次自动同步：%02d:%02d 后（上次%s）"
                                % (left // 60, left % 60,
                                   self.last_result or "—"))


def main():
    root = tk.Tk()
    try:
        # 让窗口在任务栏正常显示
        root.iconname(APP_NAME)
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
