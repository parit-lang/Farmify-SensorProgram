#!/usr/bin/env python3
"""Farmify — the sensor client and its web page in one file.

Everything FarmifySensor.py does is built in here: the HC-12 radio client, the
crop range sheet, and the assessment. Nothing else needs to be downloaded. Run
it and open the address it prints.

    python3 farmify_app.py

    this program <--USB--> Nano + HC-12 <--433 MHz--> HC-12 + Uno --> soil probe

The page has a Scan & connect button that walks the machine's USB serial ports
asking each one to identify itself, and keeps the one that answers with the
receiver's signature. After that, Start pulls live values off the probe over
the air.

Nothing about the receiver is hardcoded: no device path, and the gateway Uno's
own USB port is never opened — it is treated as a board that happens to be
plugged in for power. Every byte goes over the radio.

Needs pyserial (pip install pyserial). Everything else is the standard library.
"""

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit('farmify_app.py needs pyserial: pip install pyserial')

PORT = 8000

# ------------------------------------------------------------ reading model

REGISTER_COUNT = 7

CHANNELS = ('moisture_pct', 'temperature_c', 'ec_ds_m', 'ph',
            'nitrogen_mg_kg', 'phosphorus_mg_kg', 'potassium_mg_kg')

UNITS = {
    'moisture_pct': '%', 'temperature_c': 'C', 'ec_ds_m': 'dS/m', 'ph': '',
    'nitrogen_mg_kg': 'mg/kg', 'phosphorus_mg_kg': 'mg/kg',
    'potassium_mg_kg': 'mg/kg',
}

LABELS = {
    'moisture_pct': 'moisture', 'temperature_c': 'temperature',
    'ec_ds_m': 'ec', 'ph': 'ph', 'nitrogen_mg_kg': 'nitrogen',
    'phosphorus_mg_kg': 'phosphorus', 'potassium_mg_kg': 'potassium',
}


def to_signed(value: int) -> int:
    """Reinterpret an unsigned 16-bit register as two's-complement."""
    return value - 0x10000 if value > 0x7FFF else value


def reading_from_registers(regs: list[int]) -> dict:
    """One sweep of all seven channels, in physical units."""
    if len(regs) != REGISTER_COUNT:
        raise ValueError(f'expected {REGISTER_COUNT} registers, got {len(regs)}')
    return {
        'moisture_pct': regs[0] / 10,
        'temperature_c': to_signed(regs[1]) / 10,
        'ec_ds_m': regs[2] / 1000,   # probe reports uS/cm; 1 dS/m = 1000 uS/cm
        'ph': regs[3] / 10,
        'nitrogen_mg_kg': regs[4],
        'phosphorus_mg_kg': regs[5],
        'potassium_mg_kg': regs[6],
    }


class BridgeError(Exception):
    """The bridge was unreachable, or answered with something unusable."""


def parse_response(line: str) -> list[int]:
    """Turn one `READ` reply into raw registers."""
    line = line.strip()
    if not line:
        raise BridgeError('empty reply from bridge')
    if line.startswith('ERR'):
        raise BridgeError(line[3:].strip() or 'unspecified bridge error')
    if not line.startswith('OK'):
        raise BridgeError(f'unrecognised reply: {line!r}')

    body = line[2:].strip()
    if not body:
        raise BridgeError('OK with no register data')
    try:
        regs = [int(field) for field in body.split(',')]
    except ValueError as e:
        raise BridgeError(f'non-numeric register data: {body!r}') from e
    if len(regs) != REGISTER_COUNT:
        raise BridgeError(f'expected {REGISTER_COUNT} registers, got {len(regs)}')
    for value in regs:
        if not 0 <= value <= 0xFFFF:
            raise BridgeError(f'register out of 16-bit range: {value}')
    return regs


# ------------------------------------------------------------ radio link

PROBE = '!ID'
SIGNATURE = 'FARMIFY-HC12-RX'

DEFAULT_BAUD = 9600         # HC-12 factory default, and what the sketches use
PROBE_TIMEOUT_S = 2.0

# Opening a port asserts DTR, which pulses RESET on an Arduino. That cannot be
# avoided: with DTR held low the FTDI on a Nano does not pass traffic at all, so
# every open costs a reboot and every open has to wait one out.
RESET_SETTLE_S = 2.0

CONNECT_ATTEMPTS = 3
CONNECT_BACKOFF_S = 2.0

# A corrupted command reaches the gateway as an unknown one. The radio is a
# lossy medium and a retry is nearly always enough, so it is worth one before
# the error reaches the caller.
COMMAND_ATTEMPTS = 3


class LinkError(BridgeError):
    """The radio link itself failed: no carrier, no receiver, no reply at all.

    Separate from a plain BridgeError, which is what the gateway sends when it
    was reached perfectly well and had bad news about the probe.
    """


class ReceiverNotFound(LinkError):
    """No serial port answered the identity probe."""


def _open(device, baud, timeout):
    """Open a port and wait for the board behind it to finish rebooting."""
    port = serial.Serial(device, baud, timeout=timeout)
    time.sleep(RESET_SETTLE_S)
    port.reset_input_buffer()
    return port


def _read_for(port, seconds, until=None):
    deadline = time.monotonic() + seconds
    buf = b''
    while time.monotonic() < deadline:
        buf += port.read(256)
        if until is not None and until in buf:
            break
    return buf


def candidate_ports():
    """Serial ports worth probing, most-likely first.

    Restricted to ports with a USB vendor ID, because a PC advertises dozens of
    legacy ttyS* devices that no board is ever behind and each one costs a probe.
    """
    ports = list(serial.tools.list_ports.comports())
    usb = [info.device for info in ports if info.vid is not None]
    return usb or [info.device for info in ports]


def identifies(device, baud=DEFAULT_BAUD, timeout=PROBE_TIMEOUT_S):
    """Ask one port who it is, and say whether the receiver answered.

    Raises PermissionError rather than swallowing it. A port the user cannot
    open is a fixable problem with a specific fix, and reporting it as simply
    "not the receiver" sends people looking at their wiring instead.
    """
    try:
        with _open(device, baud, 0.2) as port:
            port.write((PROBE + '\r\n').encode())
            port.flush()
            reply = _read_for(port, timeout, until=SIGNATURE.encode())
    except PermissionError:
        raise
    except (OSError, serial.SerialException):
        return False
    return SIGNATURE.encode() in reply


def find_receiver(baud=DEFAULT_BAUD, timeout=PROBE_TIMEOUT_S):
    """Return the device path of the HC-12 receiver, or raise."""
    ports = candidate_ports()
    if not ports:
        raise ReceiverNotFound('no serial ports on this machine at all')

    denied = []
    for device in ports:
        try:
            if identifies(device, baud, timeout):
                return device
        except PermissionError:
            denied.append(device)

    if denied:
        raise ReceiverNotFound(
            'permission denied opening {}. On Linux add yourself to the dialout '
            'group (sudo usermod -aG dialout $USER) and log back in.'
            .format(', '.join(denied)))
    raise ReceiverNotFound(
        'none of {} answered the identity probe. Check the Nano is plugged in '
        'and running FarmifyHC12Rx.'.format(', '.join(ports)))


def discover():
    """Every USB serial port that could be the receiver.

    Deliberately does not probe: this only backs the /scan endpoint, and
    probing every port costs a board reset each. Same {address, name} shape the
    page has always been given, so nothing on the front end has to change.
    """
    entries = []
    for info in serial.tools.list_ports.comports():
        if info.vid is None:
            continue
        entries.append({'address': info.device,
                        'name': info.description or info.device})
    return entries


class HC12Bridge:
    """Client for the Arduino gateway, reached over the HC-12 link."""

    def __init__(self, port=None, baud=DEFAULT_BAUD, timeout=8.0,
                 attempts=CONNECT_ATTEMPTS, backoff=CONNECT_BACKOFF_S):
        self.baud = baud
        self.timeout = timeout
        self._buffer = b''
        self.port = port or find_receiver(baud)
        self._serial = self._connect(attempts, backoff)
        self._prime()

    def _connect(self, attempts, backoff):
        last = None
        for attempt in range(1, attempts + 1):
            try:
                return _open(self.port, self.baud, 0.2)
            except (OSError, serial.SerialException) as e:
                last = e
                if attempt < attempts:
                    time.sleep(backoff)
        raise LinkError(
            f'could not open {self.port} after {attempts} attempts: {last}')

    def _prime(self, attempts=3):
        """Absorb the boot noise that follows the port opening.

        Opening resets the receiver, and the first line it forwards is usually
        the tail of whatever was in flight when RESET fired. command() retries
        past that on its own, but the link is cleared once up front so the
        first real reading does not pay for it.
        """
        for _ in range(attempts):
            try:
                if self._once('PING', wait=3.0) == 'PONG':
                    return
            except BridgeError:
                pass

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def _readline(self, deadline):
        while b'\n' not in self._buffer:
            if time.monotonic() > deadline:
                raise LinkError('timed out waiting for a reply over the radio')
            chunk = self._serial.read(256)
            if not chunk:
                continue
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b'\n', 1)
        return line.decode('utf-8', 'replace').strip()

    def _once(self, text, wait):
        self._buffer = b''
        self._serial.reset_input_buffer()
        self._serial.write(text.encode() + b'\r\n')
        self._serial.flush()
        deadline = time.monotonic() + wait
        while True:
            line = self._readline(deadline)
            if line:
                return line

    def command(self, text, wait=6.0, attempts=COMMAND_ATTEMPTS):
        """Send one command and return its first non-empty reply line."""
        last = None
        for attempt in range(1, attempts + 1):
            try:
                reply = self._once(text, wait)
            except BridgeError as e:
                last = e
            else:
                if not reply.startswith('ERR unknown command'):
                    return reply
                # Corruption on the air, not a gateway opinion: worth a resend.
                last = LinkError(f'gateway did not understand {text!r}')
            if attempt < attempts:
                time.sleep(0.4)
        raise last

    def ping(self):
        return self.command('PING') == 'PONG'

    def read(self):
        last = None
        for attempt in range(1, COMMAND_ATTEMPTS + 1):
            try:
                return reading_from_registers(
                    parse_response(self.command('READ', attempts=1)))
            except BridgeError as e:
                last = e
                if attempt < COMMAND_ATTEMPTS:
                    time.sleep(0.4)
        raise last


# ------------------------------------------------------------ crop ideal ranges

# Ideal ranges transcribed from the Farmify crop sheet, in the sheet's own
# units: (low, high), None on one side meaning unbounded, and a whole channel
# of None meaning the sheet lists no target. Readings arrive in these same
# units — ppm maps 1:1 onto mg/kg, and reading_from_registers already converts
# the probe's uS/cm to dS/m. The sheet's H column is taken as soil moisture,
# since that is what this sensor measures.
_SHEET = {
    #                  moisture     temp C     EC dS/m         pH        N mg/kg        P mg/kg     K mg/kg
    'corn':           ((55, 65),   (24, 30),  (1.5, 2.5),  (6.0, 6.8),   (150, 200),   (50, 60),   (150, 200)),
    'grapes':         ((40, 50),   (25, 32),  (None, 1.0), (5.5, 6.5),   (500, 1200),  (40, 50),   (250, 300)),
    'almonds':        ((40, 60),   (24, 35),  (None, 1.5), None,         (100, 150),   (30, 50),   (150, 250)),
    'lettuce':        ((50, 70),   (13, 18),  (0.9, 1.3),  (6.0, 6.5),   (80, 150),    (35, 55),   (100, 150)),
    'tomato':         ((60, 70),   (15, 29),  (2.0, 3.5),  (6.0, 6.8),   (100, 150),   (50, 100),  (200, 300)),
    'strawberry':     ((60, 75),   (15, 27),  (1.0, 1.5),  (5.5, 6.2),   (500, 800),   (30, 50),   (125, 200)),
    'avocado':        ((60, 80),   (20, 28),  (None, 0.75),(6.2, 6.5),   (100, 150),   (30, 50),   (150, 250)),
    'soybeans':       ((50, 70),   (25, 30),  (2.8, 3.6),  (6.3, 6.5),   None,         (15, 40),   (100, 170)),
    'broccoli':       ((60, 70),   (15, 20),  (2.8, 3.5),  (6.0, 6.8),   (150, 200),   (50, 80),   (200, 300)),
    'blueberries':    ((50, 70),   (18, 29),  (0.8, 1.2),  (4.5, 5.5),   (80, 120),    (20, 30),   (80, 110)),
    'pistachios':     ((None, 35), (35, 40),  (None, 4.0), (7.0, 7.8),   (23000, 29000), (20, 40), (150, 250)),
    'rice':           ((50, 75),   (24, 35),  (1.0, 2.0),  (5.5, 6.5),   (90, 130),    (20, 40),   (60, 100)),
    'tobacco':        ((60, 70),   (21, 29),  (1.5, 2.5),  (6.0, 6.8),   (100, 150),   (35, 50),   (100, 200)),
    'cotton':         ((50, 60),   (21, 27),  (None, 1.7), (5.8, 6.5),   (50, 70),     (15, 20),   (60, 80)),
    'potatoes':       ((60, 80),   (15, 20),  (1.0, 2.0),  (5.5, 6.5),   (40, 70),     (15, 25),   (100, 150)),
    'sweet potatoes': ((50, 70),   (24, 32),  (0.8, 1.2),  (5.5, 6.8),   (50, 75),     (50, 100),  (150, 250)),
    'watermelon':     ((65, 85),   (25, 35),  (2.0, 2.5),  (6.0, 6.8),   (120, 150),   (50, 80),   (150, 200)),
    'bell peppers':   ((40, 70),   (21, 27),  (1.8, 2.5),  (6.0, 6.8),   (180, 200),   (50, 50),   (240, 300)),
}

CROP_RANGES = {crop: dict(zip(CHANNELS, row)) for crop, row in _SHEET.items()}

# Spellings and singulars that should land on the same row.
ALIASES = {
    'avacado': 'avocado', 'almond': 'almonds', 'bell pepper': 'bell peppers',
    'blueberry': 'blueberries', 'grape': 'grapes', 'pistachio': 'pistachios',
    'potato': 'potatoes', 'soybean': 'soybeans', 'strawberries': 'strawberry',
    'sweet potato': 'sweet potatoes', 'tomatoes': 'tomato',
}


def ranges_for(crop: str):
    """The sheet row for a crop name, however the user spelled it."""
    key = ' '.join(crop.lower().split())
    return CROP_RANGES.get(ALIASES.get(key, key))


def assess(reading: dict, ranges: dict) -> dict:
    """Judge each channel against its ideal range: too low, too high, or good."""
    verdicts = {}
    for name, value in reading.items():
        span = ranges.get(name)
        if span is None:
            verdicts[LABELS[name]] = 'no range listed'
            continue
        low, high = span
        if low is not None and value < low:
            verdicts[LABELS[name]] = 'too low'
        elif high is not None and value > high:
            verdicts[LABELS[name]] = 'too high'
        else:
            verdicts[LABELS[name]] = 'good'
    return verdicts


def format_reading(reading: dict) -> str:
    return '\n'.join(
        f'  {name:<18} {value:>8} {UNITS[name]}'.rstrip()
        for name, value in reading.items()
    )


def format_assessment(verdicts: dict) -> str:
    return '\n'.join(f'  {label} {verdict}' for label, verdict in verdicts.items())


# ------------------------------------------------------------ connection state

class Link:
    """The one live bridge connection, shared by every request."""

    def __init__(self):
        self._lock = threading.Lock()
        self._bridge = None
        self.device = None

    @property
    def connected(self):
        return self._bridge is not None

    def status(self):
        return {'connected': self.connected, 'device': self.device}

    def connect(self, address=None):
        with self._lock:
            self._drop()
            port = address or find_receiver()
            device = {'address': port, 'name': SIGNATURE}
            self._bridge = HC12Bridge(port)
            self.device = device
            return device

    def read(self):
        with self._lock:
            if self._bridge is None:
                raise BridgeError('not connected — press Scan & connect first')
            try:
                return self._bridge.read()
            except BridgeError:
                self._drop()
                raise

    def disconnect(self):
        with self._lock:
            self._drop()

    def _drop(self):
        if self._bridge is not None:
            self._bridge.close()
            self._bridge = None
        self.device = None


LINK = Link()


# ------------------------------------------------------------ web page

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Farmify Sensor</title>
<style>
  :root { color-scheme: light dark; }
  * { box-sizing: border-box; }
  body {
    font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 580px; margin: 2.5rem auto; padding: 0 1.25rem;
    background: #f7f8f6; color: #1c2118;
  }
  header { display: flex; align-items: flex-start; gap: 1rem; margin-bottom: 1.5rem; flex-wrap: wrap; }
  .titles { flex: 1; min-width: 12rem; }
  h1 { font-size: 1.4rem; margin: 0 0 .3rem; }
  .sub { margin: 0; font-size: .85rem; color: #6d7566; }
  .box {
    background: #fff; border: 1px solid #d9ded4; border-radius: 10px;
    padding: 1rem 1.15rem; margin-bottom: 1rem;
  }
  label, .lbl { display: block; font-weight: 600; font-size: .9rem; margin-bottom: .5rem; }
  input {
    width: 100%; padding: .6rem .7rem; font-size: 1rem;
    border: 1px solid #c6cdbf; border-radius: 6px;
    background: #fdfdfc; color: inherit; font-family: inherit;
  }
  input:focus { outline: 2px solid #307A9C; outline-offset: 1px; }
  button {
    font-family: inherit; font-weight: 600; border: 0; border-radius: 6px;
    background: #307A9C; color: #fff; cursor: pointer;
  }
  button:hover:not(:disabled) { background: #0F5373; }
  button:disabled { opacity: .55; cursor: default; }
  #start { width: 100%; padding: .85rem; font-size: 1rem; min-height: 44px; }
  #bt, #dis { padding: .6rem .9rem; font-size: .85rem; white-space: nowrap; min-height: 44px; }
  #dis { margin-left: .35rem; background: transparent; color: #6d7566;
         border: 1px solid #c6cdbf; }
  #dis:hover:not(:disabled) { background: #f0f2ee; color: #a4342a; border-color: #c9a19b; }
  .btwrap { text-align: right; }
  [hidden] { display: none !important; }
  .pill {
    display: inline-block; margin-top: .4rem; font-size: .74rem;
    color: #6d7566; max-width: 15rem;
  }
  .dot { display: inline-block; width: .5rem; height: .5rem; border-radius: 50%;
         background: #b3bba9; margin-right: .3rem; vertical-align: middle; }
  .dot.on { background: #4a7c3f; }
  .dot.bad { background: #a4342a; }
  pre {
    margin: 0; white-space: pre-wrap; word-break: break-word;
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: .875rem; line-height: 1.5; min-height: 1.2em;
  }
  .muted { color: #6d7566; }
  .err { color: #a4342a; }
  .site-footer {
    display: flex; align-items: center; gap: .85rem; flex-wrap: wrap;
    margin-top: 1.5rem; padding-top: 1rem; border-top: 1px solid #d9ded4;
    font-size: .8rem; color: #6d7566;
  }
  .footer-logo {
    width: 2rem; height: 2rem; flex-shrink: 0; object-fit: contain;
  }
  .footer-info p { margin: 0 0 .2rem; }
  .footer-info a { color: #307A9C; text-decoration: none; }
  .footer-info a:hover { color: #0F5373; text-decoration: underline; }
  @media (prefers-color-scheme: dark) {
    body { background: #14170f; color: #e6e9e0; }
    .box { background: #1d2117; border-color: #333a2b; }
    input { background: #14170f; border-color: #3d4534; }
    #dis { color: #8d9683; border-color: #3d4534; }
    #dis:hover:not(:disabled) { background: #262b1e; color: #e88b7f; border-color: #5a4038; }
    .muted, .sub, .pill { color: #8d9683; }
    .err { color: #e88b7f; }
    .site-footer { border-color: #333a2b; color: #8d9683; }
    .footer-info a { color: #6fb3d9; }
    .footer-info a:hover { color: #9ccbe6; }
  }
  @media (max-width: 480px) {
    body { margin: 1.25rem auto; padding: 0 1rem; }
    header { flex-direction: column; align-items: stretch; gap: .75rem; margin-bottom: 1.15rem; }
    .btwrap { display: flex; flex-wrap: wrap; gap: .5rem; text-align: left; }
    #bt, #dis { flex: 1; margin-left: 0; }
    .pill { flex-basis: 100%; max-width: 100%; margin-top: .2rem; }
    .box { padding: .9rem 1rem; }
  }
</style>
</head>
<body>

<header>
  <div class="titles">
    <h1>Farmify Sensor</h1>
    <p class="sub">Live soil readings over Bluetooth.</p>
  </div>
  <div class="btwrap">
    <button id="bt">Scan &amp; connect</button><button id="dis" hidden>Disconnect</button>
    <div class="pill"><span class="dot" id="dot"></span><span id="status">Not connected</span></div>
  </div>
</header>

<div class="box">
  <label for="crop">Enter crop name</label>
  <input id="crop" list="crops" placeholder="e.g. Corn" autocomplete="off" autofocus>
  <datalist id="crops"></datalist>
</div>

<div class="box">
  <button id="start">Start</button>
</div>

<div class="box">
  <span class="lbl">Result</span>
  <pre id="result" class="muted">No reading yet.</pre>
</div>

<footer class="site-footer">
  <img class="footer-logo" src="assets/logo.png" alt="Farmify logo">
  <div class="footer-info">
    <p><a href="https://www.farmify.us" target="_blank" rel="noopener">www.farmify.us</a></p>
    <p>contact@farmify.example &middot; (555) 012-3456</p>
  </div>
</footer>

<script>
'use strict';

const cropEl = document.getElementById('crop');
const btBtn = document.getElementById('bt');
const disBtn = document.getElementById('dis');
const startBtn = document.getElementById('start');
const dot = document.getElementById('dot');
const statusEl = document.getElementById('status');
const out = document.getElementById('result');

document.getElementById('crops').innerHTML =
  CROPS.map(c => `<option value="${c}">`).join('');

function show(text, isError) {
  out.className = isError ? 'err' : '';
  out.textContent = text;
}

function setStatus(state, text) {
  dot.className = 'dot' + (state ? ' ' + state : '');
  statusEl.textContent = text;
}

async function post(path, body) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
  return res.json();
}

function applyStatus(s) {
  disBtn.hidden = !s.connected;
  if (s.connected) {
    const d = s.device || {};
    setStatus('on', `${d.name || 'connected'} (${d.address || ''})`);
    btBtn.textContent = 'Reconnect';
  } else {
    setStatus('', 'Not connected');
    btBtn.textContent = 'Scan & connect';
  }
}

async function disconnect() {
  disBtn.disabled = true;
  try {
    const r = await post('/disconnect');
    applyStatus(r.status);
    show('Disconnected. The module is free for another device.', false);
  } catch (e) {
    show('Could not reach the app: ' + e.message, true);
  } finally {
    disBtn.disabled = false;
  }
}

async function scanConnect() {
  btBtn.disabled = true;
  setStatus('', 'Scanning…');
  try {
    const r = await post('/connect');
    if (r.ok) {
      applyStatus(r.status);
      show(`Connected to ${r.status.device.name} (${r.status.device.address}).`
           + `\nEnter a crop name and press Start.`, false);
    } else {
      setStatus('bad', 'Not connected');
      btBtn.textContent = 'Scan & connect';
      show(r.error, true);
    }
  } catch (e) {
    setStatus('bad', 'Not connected');
    show('Could not reach the app: ' + e.message, true);
  } finally {
    btBtn.disabled = false;
  }
}

async function start() {
  const crop = cropEl.value.trim();
  if (!crop) { show('Enter a crop name first.', true); cropEl.focus(); return; }

  startBtn.disabled = true;
  out.className = 'muted';
  out.textContent = 'Reading ' + crop + '…';
  try {
    const r = await post('/read', { crop });
    applyStatus(r.status);
    show(r.output, !r.ok);
  } catch (e) {
    show('Could not reach the app: ' + e.message, true);
  } finally {
    startBtn.disabled = false;
  }
}

btBtn.addEventListener('click', scanConnect);
disBtn.addEventListener('click', disconnect);
startBtn.addEventListener('click', start);
cropEl.addEventListener('keydown', e => { if (e.key === 'Enter') start(); });

post('/status').then(applyStatus).catch(() => {});
</script>
</body>
</html>
"""


def page() -> str:
    crops = json.dumps(sorted(CROP_RANGES))
    return PAGE.replace('<script>\n\'use strict\';',
                        f'<script>\n\'use strict\';\nconst CROPS = {crops};', 1)


# ------------------------------------------------------------ http

def do_read(crop: str) -> dict:
    if not crop:
        return {'ok': False, 'output': 'no crop type given'}
    try:
        reading = LINK.read()
    except BridgeError as e:
        return {'ok': False, 'output': f'HC-12 link: {e}'}

    ranges = ranges_for(crop)
    body = [f'Crop: {crop}', format_reading(reading), '']
    body.append(format_assessment(assess(reading, ranges)) if ranges
                else '  crop not in our dataset sorry')
    return {'ok': True, 'output': '\n'.join(body)}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, payload):
        self._send(200, json.dumps(payload), 'application/json')

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, page(), 'text/html; charset=utf-8')
        else:
            self._send(404, 'not found', 'text/plain')

    def do_POST(self):
        length = int(self.headers.get('Content-Length') or 0)
        try:
            payload = json.loads(self.rfile.read(length) or b'{}')
        except json.JSONDecodeError:
            payload = {}

        if self.path == '/status':
            self._json(LINK.status())
        elif self.path == '/scan':
            self._json({'devices': discover()})
        elif self.path == '/connect':
            try:
                LINK.connect(payload.get('address'))
                self._json({'ok': True, 'status': LINK.status()})
            except BridgeError as e:
                self._json({'ok': False, 'error': str(e), 'status': LINK.status()})
        elif self.path == '/disconnect':
            LINK.disconnect()
            self._json({'ok': True, 'status': LINK.status()})
        elif self.path == '/read':
            result = do_read(str(payload.get('crop', '')).strip())
            self._json({**result, 'status': LINK.status()})
        else:
            self._send(404, 'not found', 'text/plain')

    def log_message(self, fmt, *args):
        pass


def main() -> int:
    print(f'Farmify -> http://localhost:{PORT}   (Ctrl-C to stop)')
    server = ThreadingHTTPServer(('127.0.0.1', PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopped')
    finally:
        LINK.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
