# SPDX-License-Identifier: GPL-3.0-or-later
"""Wi-Fi link parsing and roam tracking for wifi_monitor (pure, unit tested).

2026-11-01. The mower's bcmdhd firmware roams on its own under NetworkManager and the
stack had no record of the link. These helpers turn the root-free sources into one
sample dict and track BSSID changes across samples:

* ``/proc/net/wireless``        link quality, signal dBm
* ``iw dev <iface> link``       BSSID, SSID, freq, tx/rx bitrate
* ``iw dev <iface> station dump`` tx retries / tx failed (cumulative counters)
* ``/sys/class/net/<iface>/operstate``

Review 2026-11-02: an ``iw`` that times out or fails (load avg 12 on the mower) is not a
disconnect. ``build_sample`` labels such a sample ``unknown``, as it does one where ``iw``
says "Not connected." while /proc/net/wireless still shows a signal; ``LinkFilter`` keeps
the last known sample for ``stale_samples`` of them and ``RoamTracker`` never turns an
unknown sample into an event. A disconnect is logged when the link is really gone: operstate
down, or ``iw`` not connected AND no signal in /proc/net/wireless (or ``iw`` not connected
for more than ``stale_samples`` samples in a row).

2026-11-03: when ``iw`` is absent, or every call failed for more than ``stale_samples``
samples, ``build_proc_sample`` reports from /proc/net/wireless + operstate + carrier alone
(``source`` 'proc': connected with the /proc signal, no BSSID/SSID/bitrates). Samples carry
``source`` 'iw' otherwise; RoamTracker never counts a roam across a 'proc' sample (no BSSID).
"""

import re

_NUM = r'(-?\d+(?:\.\d+)?)'


def parse_proc_wireless(text, iface):
    """``/proc/net/wireless`` -> {'link_quality': float, 'signal_dbm': float} or None."""
    for line in (text or '').splitlines():
        if ':' not in line:
            continue
        name, _, rest = line.partition(':')
        if name.strip() != iface:
            continue
        fields = rest.split()
        if len(fields) < 3:
            return None
        try:
            quality = float(fields[1].rstrip('.'))
            level = float(fields[2].rstrip('.'))
        except ValueError:
            return None
        # Old drivers report level as unsigned 8-bit (e.g. 200 = -56 dBm).
        if level > 0:
            level -= 256
        return {'link_quality': quality, 'signal_dbm': level}
    return None


def parse_iw_link(text):
    """``iw dev <iface> link`` -> dict. ``connected`` False on 'Not connected.'/empty."""
    out = {'connected': False, 'bssid': '', 'ssid': '', 'freq_mhz': 0.0,
           'signal_dbm': None, 'tx_bitrate_mbps': 0.0, 'rx_bitrate_mbps': 0.0}
    text = text or ''
    m = re.search(r'^Connected to ([0-9a-fA-F:]{17})', text, re.M)
    if not m:
        return out
    out['connected'] = True
    out['bssid'] = m.group(1).lower()
    for key, pattern in (('ssid', r'^\s*SSID:\s?(.*)$'),):
        mm = re.search(pattern, text, re.M)
        if mm:
            out[key] = mm.group(1).strip()
    for key, pattern in (('freq_mhz', r'^\s*freq:\s*' + _NUM),
                         ('signal_dbm', r'^\s*signal:\s*' + _NUM + r'(?:\s*\[[^]]*\])?\s*dBm'),
                         ('tx_bitrate_mbps', r'^\s*tx bitrate:\s*' + _NUM + r'\s*MBit/s'),
                         ('rx_bitrate_mbps', r'^\s*rx bitrate:\s*' + _NUM + r'\s*MBit/s')):
        mm = re.search(pattern, text, re.M)
        if mm:
            out[key] = float(mm.group(1))
    return out


def parse_station_dump(text):
    """``iw dev <iface> station dump`` -> {'tx_retries': int, 'tx_failed': int} or None.

    A client normally has one station (its AP); counters of several are summed.
    """
    retries = failed = 0
    found = False
    for line in (text or '').splitlines():
        m = re.match(r'^\s*tx (retries|failed):\s*(\d+)', line)
        if m:
            found = True
            if m.group(1) == 'retries':
                retries += int(m.group(2))
            else:
                failed += int(m.group(2))
    return {'tx_retries': retries, 'tx_failed': failed} if found else None


def freq_to_channel(freq_mhz):
    """802.11 channel for a centre frequency in MHz (0 when unknown)."""
    f = int(round(freq_mhz or 0))
    if f == 2484:
        return 14
    if 2412 <= f <= 2472:
        return (f - 2407) // 5
    if 5000 <= f <= 5895:
        return (f - 5000) // 5
    if 5955 <= f <= 7115:
        return (f - 5950) // 5
    return 0


def link_state(connected, operstate, signal_dbm):
    """Short label: down | disconnected | bad | weak | ok | good (build_sample adds
    unknown)."""
    if operstate not in ('up', 'unknown', 'dormant', '') and not connected:
        return 'down'
    if not connected:
        return 'disconnected'
    if signal_dbm is None:
        return 'ok'
    if signal_dbm > -60:
        return 'good'
    if signal_dbm >= -70:
        return 'ok'
    if signal_dbm >= -78:
        return 'weak'
    return 'bad'


def build_sample(iface, proc_text, link_text, operstate, station=None):
    """Combine the raw sources into one sample dict (all fields always present).

    ``link_text`` None = ``iw`` failed or timed out. Such a sample, and one where ``iw``
    says not connected while /proc/net/wireless has a signal, gets link_state ``unknown``
    (connected False) unless operstate is down; ``iw_said_not_connected`` tells the two
    apart (LinkFilter)."""
    link = parse_iw_link(link_text)
    proc = parse_proc_wireless(proc_text, iface)
    operstate = (operstate or '').strip()
    connected = link['connected'] and operstate != 'down'
    proc_signal = proc is not None and proc['signal_dbm'] < 0.0
    unknown = operstate != 'down' and not link['connected'] and (
        link_text is None or proc_signal)
    signal = link['signal_dbm']
    if signal is None and proc is not None and connected:
        signal = proc['signal_dbm']
    sample = {
        'iface': iface, 'connected': connected, 'operstate': operstate,
        'ssid': link['ssid'] if connected else '',
        'bssid': link['bssid'] if connected else '',
        'freq_mhz': link['freq_mhz'] if connected else 0.0,
        'signal_dbm': signal if connected else None,
        'link_quality': proc['link_quality'] if proc else 0.0,
        'tx_bitrate_mbps': link['tx_bitrate_mbps'] if connected else 0.0,
        'rx_bitrate_mbps': link['rx_bitrate_mbps'] if connected else 0.0,
        'tx_retries': station['tx_retries'] if station else 0,
        'tx_failed': station['tx_failed'] if station else 0,
    }
    sample['link_state'] = 'unknown' if unknown else \
        link_state(connected, operstate, sample['signal_dbm'])
    sample['iw_said_not_connected'] = link_text is not None and not link['connected']
    sample['source'] = 'iw'
    return sample


def build_proc_sample(iface, proc_text, operstate, carrier):
    """Signal-only sample when ``iw`` is unavailable: connected when /proc/net/wireless has a
    signal, operstate is not down and carrier is not '0'; link_state from the signal."""
    proc = parse_proc_wireless(proc_text, iface)
    operstate = (operstate or '').strip()
    carrier = (carrier or '').strip()
    connected = proc is not None and proc['signal_dbm'] < 0.0 and operstate != 'down' and \
        carrier != '0'
    signal = proc['signal_dbm'] if connected else None
    return {
        'iface': iface, 'connected': connected, 'operstate': operstate,
        'ssid': '', 'bssid': '', 'freq_mhz': 0.0, 'signal_dbm': signal,
        'link_quality': proc['link_quality'] if proc else 0.0,
        'tx_bitrate_mbps': 0.0, 'rx_bitrate_mbps': 0.0, 'tx_retries': 0, 'tx_failed': 0,
        'link_state': link_state(connected, operstate, signal),
        'iw_said_not_connected': False, 'source': 'proc',
    }


class LinkFilter:
    """Bridges ``unknown`` samples (build_sample): the last known sample is reported for
    up to ``stale_samples`` consecutive unknown ones; after that the unknown sample itself
    (connected False, link_state unknown) -- or, when ``iw`` kept saying not connected all
    along, a plain disconnected one."""

    def __init__(self, stale_samples=3):
        self._stale_samples = max(0, int(stale_samples))
        self._known = None
        self._unknown_run = 0
        self._iw_not_connected_run = 0

    def update(self, sample):
        if sample['link_state'] != 'unknown':
            self._known = sample
            self._unknown_run = self._iw_not_connected_run = 0
            return sample
        self._unknown_run += 1
        self._iw_not_connected_run = self._iw_not_connected_run + 1 \
            if sample['iw_said_not_connected'] else 0
        if self._known is not None and self._unknown_run <= self._stale_samples:
            return self._known
        if self._iw_not_connected_run > self._stale_samples:
            return dict(sample, link_state=link_state(False, sample['operstate'], None))
        return sample


class RoamTracker:
    """Counts roams and emits events across consecutive samples.

    Events: ``roam`` (BSSID changed while connected), ``disconnect``, ``reconnect``
    (``connect`` for the first connection). Each carries the old and new BSSID, freq and
    signal. The counter only grows for ``roam``.
    """

    def __init__(self):
        self.roam_count = 0
        self._last = None
        self._ever_connected = False

    def update(self, sample):
        """Feed one sample; returns the list of event dicts it caused. An ``unknown``
        sample (build_sample) causes none and is not remembered."""
        if sample['link_state'] == 'unknown':
            return []
        prev = self._last
        events = []
        if sample['connected']:
            if prev is None or not prev['connected']:
                kind = 'reconnect' if self._ever_connected else 'connect'
                events.append(self._event(kind, prev, sample))
            elif prev['bssid'] != sample['bssid'] and \
                    prev.get('source', 'iw') == sample.get('source', 'iw') == 'iw':
                self.roam_count += 1
                events.append(self._event('roam', prev, sample))
            self._ever_connected = True
        elif prev is not None and prev['connected']:
            events.append(self._event('disconnect', prev, sample))
        if sample['connected'] or prev is None or prev['connected']:
            self._last = dict(sample)
        return events

    def _event(self, kind, prev, cur):
        def side(s):
            if not s or not s['connected']:
                return {'bssid': '', 'freq_mhz': 0.0, 'signal_dbm': None}
            return {'bssid': s['bssid'], 'freq_mhz': s['freq_mhz'],
                    'signal_dbm': s['signal_dbm']}
        return {'event': kind, 'iface': cur['iface'], 'ssid': cur['ssid'] or
                (prev or {}).get('ssid', ''), 'old': side(prev), 'new': side(cur),
                'roam_count': self.roam_count}
