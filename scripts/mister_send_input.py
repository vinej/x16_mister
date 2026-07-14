#!/usr/bin/env python3
"""
mister_send_input.py - stream the host PC keyboard (and mouse / joystick)
to a MiSTer over the network, with ZERO changes to the MiSTer.

How it works (fully transparent, works on any stock MiSTer):
  * Opens an SSH session to the MiSTer (dropbear is on by default, root/1).
  * Starts a tiny receiver ON THE FLY via `python3 -c ...` -- Python 3 ships
    with stock MiSTer Linux (the official downloader needs it).  NOTHING is
    written to the MiSTer's SD card; nothing keeps running after you quit.
  * The receiver creates virtual input devices via /dev/uinput.  MiSTer Main
    hot-plugs them exactly like real USB devices, so the core sees ordinary
    keyboard/mouse/gamepad input.
  * This script captures your PC keyboard/mouse (pynput) and an optional USB
    joystick (pygame) and streams the events over the SSH channel.

X16 core note: the keyboard path is fully supported today.  The mouse and
gamepad devices are created and MiSTer sees them, but the X16 core does not
consume mouse/joystick yet -- they are plumbing for future core features.

Usage:
    python mister_send_input.py <mister-ip> [--password 1] [--debug]
        [--passthrough] [--no-keyboard] [--no-mouse] [--no-joystick]

Press PAUSE/BREAK to quit (releases everything, MiSTer untouched).
F12 is forwarded to the MiSTer (it opens the OSD menu there).

Requires:  pip install paramiko pynput
Optional:  pip install pygame     (only for USB joystick forwarding)
"""

import argparse
import base64
import struct
import sys
import threading
import time

import paramiko
from pynput import keyboard, mouse
from pynput.keyboard import Key

# =============================================================================
# The receiver that runs (in memory only) on the MiSTer.  Pure stdlib.
# Frames on stdin:  'K' code state          (keyboard, Linux keycode)
#                   'M' dx dy wheel buttons (mouse, int8 x3 + bitmask)
#                   'J' lo hi x y           (gamepad: 16-bit buttons + axes)
# =============================================================================
RECEIVER = r'''
import os, sys, struct, fcntl
UI_SET_EVBIT,UI_SET_KEYBIT,UI_SET_RELBIT,UI_SET_ABSBIT=0x40045564,0x40045565,0x40045566,0x40045567
UI_DEV_CREATE,UI_DEV_DESTROY=0x5501,0x5502
EV_SYN,EV_KEY,EV_REL,EV_ABS=0,1,2,3
REL_X,REL_Y,REL_WHEEL=0,1,8
BTNS_M=(0x110,0x111,0x112)                       # left, right, middle
BTNS_J=(0x130,0x131,0x133,0x134,0x136,0x137,0x13a,0x13b)  # A B X Y TL TR SEL STA
ABS_X,ABS_Y=0,1
def mkdev(name,keys=(),rels=(),abss=()):
    fd=os.open('/dev/uinput',os.O_WRONLY)
    if keys:
        fcntl.ioctl(fd,UI_SET_EVBIT,EV_KEY)
        for k in keys: fcntl.ioctl(fd,UI_SET_KEYBIT,k)
    if rels:
        fcntl.ioctl(fd,UI_SET_EVBIT,EV_REL)
        for r in rels: fcntl.ioctl(fd,UI_SET_RELBIT,r)
    if abss:
        fcntl.ioctl(fd,UI_SET_EVBIT,EV_ABS)
        for a in abss: fcntl.ioctl(fd,UI_SET_ABSBIT,a)
    mx=[0]*64; mn=[0]*64
    for a in abss: mx[a]=127; mn[a]=-127
    dev=struct.pack('80s4HI',name.encode(),3,0x16c0,0x05df,1,0)
    dev+=struct.pack('64i',*mx)+struct.pack('64i',*mn)+b'\0'*512
    os.write(fd,dev); fcntl.ioctl(fd,UI_DEV_CREATE)
    return fd
def emit(fd,t,c,v): os.write(fd,struct.pack('llHHi',0,0,t,c,v))
def syn(fd): emit(fd,0,0,0)
kbd=mkdev('PC Remote Keyboard',keys=range(1,128))
mou=mkdev('PC Remote Mouse',keys=BTNS_M,rels=(REL_X,REL_Y,REL_WHEEL))
joy=mkdev('PC Remote Gamepad',keys=BTNS_J,abss=(ABS_X,ABS_Y))
sys.stdout.write('READY\n'); sys.stdout.flush()
rd=sys.stdin.buffer
mbtn=0; jbtn=0
def s8(b): return b-256 if b>127 else b
try:
    while True:
        t=rd.read(1)
        if not t: break
        if t==b'K':
            d=rd.read(2)
            if len(d)<2: break
            emit(kbd,EV_KEY,d[0],d[1]); syn(kbd)
        elif t==b'M':
            d=rd.read(4)
            if len(d)<4: break
            dx,dy,wh,nb=s8(d[0]),s8(d[1]),s8(d[2]),d[3]
            if dx: emit(mou,EV_REL,REL_X,dx)
            if dy: emit(mou,EV_REL,REL_Y,dy)
            if wh: emit(mou,EV_REL,REL_WHEEL,wh)
            global_change=nb^mbtn
            for i,b in enumerate(BTNS_M):
                if global_change&(1<<i): emit(mou,EV_KEY,b,(nb>>i)&1)
            mbtn=nb; syn(mou)
        elif t==b'J':
            d=rd.read(4)
            if len(d)<4: break
            nb=d[0]|(d[1]<<8)
            for i,b in enumerate(BTNS_J):
                if (nb^jbtn)&(1<<i): emit(joy,EV_KEY,b,(nb>>i)&1)
            jbtn=nb
            emit(joy,EV_ABS,ABS_X,s8(d[2])); emit(joy,EV_ABS,ABS_Y,s8(d[3])); syn(joy)
except Exception:
    pass
for fd in (kbd,mou,joy):
    try: fcntl.ioctl(fd,UI_DEV_DESTROY); os.close(fd)
    except Exception: pass
'''

# =============================================================================
# PC-side key mapping: pynput -> Linux input keycodes (physical US layout)
# =============================================================================
K = {  # characters (unshifted base keys)
    'a':30,'b':48,'c':46,'d':32,'e':18,'f':33,'g':34,'h':35,'i':23,'j':36,
    'k':37,'l':38,'m':50,'n':49,'o':24,'p':25,'q':16,'r':19,'s':31,'t':20,
    'u':22,'v':47,'w':17,'x':45,'y':21,'z':44,
    '1':2,'2':3,'3':4,'4':5,'5':6,'6':7,'7':8,'8':9,'9':10,'0':11,
    '-':12,'=':13,'[':26,']':27,'\\':43,';':39,"'":40,'`':41,',':51,'.':52,
    '/':53,' ':57,
}
SHIFTED = {  # shifted char -> its physical base key (shift itself is forwarded)
    '!':'1','@':'2','#':'3','$':'4','%':'5','^':'6','&':'7','*':'8','(':'9',
    ')':'0','_':'-','+':'=','{':'[','}':']','|':'\\',':':';','"':"'",'<':',',
    '>':'.','?':'/','~':'`',
}
SPECIAL = {
    Key.space:57, Key.tab:15, Key.enter:28, Key.backspace:14, Key.esc:1,
    Key.shift:42, Key.shift_l:42, Key.shift_r:54,
    Key.ctrl:29, Key.ctrl_l:29, Key.ctrl_r:97,
    Key.alt:56, Key.alt_l:56, Key.alt_r:100, Key.alt_gr:100,
    Key.caps_lock:58,
    Key.cmd:125, Key.cmd_l:125, Key.cmd_r:126, Key.menu:127,
    Key.f1:59, Key.f2:60, Key.f3:61, Key.f4:62, Key.f5:63, Key.f6:64,
    Key.f7:65, Key.f8:66, Key.f9:67, Key.f10:68, Key.f11:87, Key.f12:88,
    Key.up:103, Key.down:108, Key.left:105, Key.right:106,
    Key.home:102, Key.end:107, Key.page_up:104, Key.page_down:109,
    Key.insert:110, Key.delete:111,
}

QUIT_KEY = Key.pause   # NOT F12: on MiSTer F12 = the OSD menu, so it must be forwarded
MOUSE_HZ = 60
JOY_HZ   = 60
JOY_AXIS_THRESHOLD = 0.5


def key_to_code(key):
    if isinstance(key, Key):
        return SPECIAL.get(key)
    ch = getattr(key, 'char', None)
    if ch is None:
        return None
    ch_l = ch.lower()
    if ch_l in K:
        return K[ch_l]
    if ch in SHIFTED:
        return K[SHIFTED[ch]]
    return None


class Link:
    """SSH channel to the in-memory receiver on the MiSTer."""

    def __init__(self, host, password):
        self.cli = paramiko.SSHClient()
        self.cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        print(f"[+] connecting to root@{host} ...")
        self.cli.connect(host, username='root', password=password,
                         look_for_keys=False, allow_agent=False, timeout=10)
        b64 = base64.b64encode(RECEIVER.encode()).decode()
        cmd = f"python3 -u -c \"import base64;exec(base64.b64decode('{b64}'))\""
        self.chan = self.cli.get_transport().open_session()
        self.chan.exec_command(cmd)
        # wait for READY from the receiver
        buf = b''
        t0 = time.time()
        while b'READY' not in buf:
            if self.chan.recv_ready():
                buf += self.chan.recv(64)
            if self.chan.exit_status_ready() or time.time() - t0 > 10:
                err = self.chan.recv_stderr(4096).decode(errors='replace')
                raise RuntimeError(f"receiver failed to start: {err.strip()}")
            time.sleep(0.05)
        print("[+] receiver running on the MiSTer (in memory only)")
        self.lock = threading.Lock()

    def send(self, frame):
        with self.lock:
            self.chan.sendall(frame)

    def close(self):
        try:
            self.chan.close()
            self.cli.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('host', help='MiSTer IP / hostname')
    ap.add_argument('--password', default='1', help='SSH password (default 1)')
    ap.add_argument('--debug', action='store_true')
    ap.add_argument('--passthrough', action='store_true',
                    help='do NOT swallow input on the PC side')
    ap.add_argument('--no-keyboard', action='store_true')
    ap.add_argument('--no-mouse', action='store_true')
    ap.add_argument('--no-joystick', action='store_true')
    args = ap.parse_args()

    link = Link(args.host, args.password)
    quit_ev = threading.Event()
    pressed = set()

    # ---- keyboard -----------------------------------------------------------
    def on_key(key, state):
        if key == QUIT_KEY:
            quit_ev.set()
            return
        code = key_to_code(key)
        if code is None:
            if args.debug:
                print(f"[k] unmapped: {key!r}")
            return
        # de-dupe auto-repeat of the make event
        if state:
            if code in pressed:
                return
            pressed.add(code)
        else:
            pressed.discard(code)
        if args.debug:
            print(f"[k] {key!r} -> {code} {'dn' if state else 'up'}")
        link.send(struct.pack('cBB', b'K', code, 1 if state else 0))

    kb_listener = None
    if not args.no_keyboard:
        kb_listener = keyboard.Listener(
            on_press=lambda k: on_key(k, True),
            on_release=lambda k: on_key(k, False),
            suppress=not args.passthrough)
        kb_listener.start()

    # ---- mouse (accumulate deltas, ship at MOUSE_HZ) ------------------------
    m_state = {'dx': 0.0, 'dy': 0.0, 'wheel': 0, 'btn': 0, 'last': None}
    m_lock = threading.Lock()

    def on_move(x, y):
        with m_lock:
            if m_state['last'] is not None:
                m_state['dx'] += x - m_state['last'][0]
                m_state['dy'] += y - m_state['last'][1]
            m_state['last'] = (x, y)

    def on_click(x, y, button, is_down):
        bit = {mouse.Button.left: 1, mouse.Button.right: 2,
               mouse.Button.middle: 4}.get(button, 0)
        with m_lock:
            if is_down:
                m_state['btn'] |= bit
            else:
                m_state['btn'] &= ~bit

    def on_scroll(x, y, sdx, sdy):
        with m_lock:
            m_state['wheel'] += sdy

    ms_listener = None
    if not args.no_mouse:
        ms_listener = mouse.Listener(on_move=on_move, on_click=on_click,
                                     on_scroll=on_scroll,
                                     suppress=not args.passthrough)
        ms_listener.start()

        def mouse_pump():
            clamp = lambda v: max(-127, min(127, int(v)))
            while not quit_ev.is_set():
                time.sleep(1.0 / MOUSE_HZ)
                with m_lock:
                    dx, dy = clamp(m_state['dx']), clamp(m_state['dy'])
                    wh, btn = clamp(m_state['wheel']), m_state['btn']
                    m_state['dx'] -= dx
                    m_state['dy'] -= dy
                    m_state['wheel'] -= wh
                    dirty = dx or dy or wh or (btn != m_state.get('pbtn'))
                    m_state['pbtn'] = btn
                if dirty:
                    link.send(struct.pack('cBBBB', b'M', dx & 0xFF, dy & 0xFF,
                                          wh & 0xFF, btn))
        threading.Thread(target=mouse_pump, daemon=True).start()

    # ---- joystick (optional, via pygame) ------------------------------------
    if not args.no_joystick:
        def joy_pump():
            try:
                import pygame
                pygame.init()
                pygame.joystick.init()
                if pygame.joystick.get_count() == 0:
                    print("[j] no USB joystick found (skipping)")
                    return
                js = pygame.joystick.Joystick(0)
                js.init()
                print(f"[j] forwarding joystick: {js.get_name()}")
            except Exception as e:
                print(f"[j] pygame unavailable ({e}) -- joystick disabled")
                return
            prev = None
            while not quit_ev.is_set():
                time.sleep(1.0 / JOY_HZ)
                pygame.event.pump()
                btn = 0
                for i in range(min(js.get_numbuttons(), 16)):
                    if js.get_button(i):
                        btn |= 1 << i
                ax = int(max(-1, min(1, js.get_axis(0))) * 127)
                ay = int(max(-1, min(1, js.get_axis(1))) * 127)
                state = (btn, ax, ay)
                if state != prev:
                    prev = state
                    link.send(struct.pack('cBBBB', b'J', btn & 0xFF,
                                          (btn >> 8) & 0xFF, ax & 0xFF, ay & 0xFF))
        threading.Thread(target=joy_pump, daemon=True).start()

    print("[+] streaming.  Press PAUSE/BREAK to quit (F12 = MiSTer OSD).")
    try:
        while not quit_ev.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    print("[+] shutting down (MiSTer left untouched)")
    if kb_listener:
        kb_listener.stop()
    if ms_listener:
        ms_listener.stop()
    link.close()


if __name__ == '__main__':
    main()
