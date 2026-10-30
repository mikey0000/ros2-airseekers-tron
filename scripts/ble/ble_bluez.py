"""Minimal BlueZ GATT client over D-Bus for ble_link_test.py (dbus-python + PyGObject).

Both ship with most desktop Linux distributions (Debian/Ubuntu: python3-dbus, python3-gi).
Only the vendor service is touched: notifications and writes on the serial pipe
(notify on c1db0002, writes to c1db0004), a read of c1db0003.
"""
import os
import time

SERVICE_UUID = "c1db0001-431d-4cbe-96e2-9f9286c41572"
# Direction split confirmed 2026-10-30: the module notifies UART bytes on c1db0002 and
# forwards phone writes on c1db0004 to the UART (writes to c1db0002 are swallowed). The
# vendor app (blueManager.dart) pairs C1DB0002 with notify and C1DB0004 with write.
PIPE_UUID = "c1db0002-431d-4cbe-96e2-9f9286c41572"     # mower -> phone (notify)
WRITE_UUID = "c1db0004-431d-4cbe-96e2-9f9286c41572"    # phone -> mower (write)
INFO_UUID = "c1db0003-431d-4cbe-96e2-9f9286c41572"
CONFIG_UUID = WRITE_UUID  # kept for callers that used the old name

BLUEZ = "org.bluez"
DEV_IF = "org.bluez.Device1"
CHAR_IF = "org.bluez.GattCharacteristic1"
PROPS_IF = "org.freedesktop.DBus.Properties"
OM_IF = "org.freedesktop.DBus.ObjectManager"


class BluezTransport:
    """``on_rx(bytes)`` is called for every notification while ``pump()`` runs."""

    def __init__(self, adapter="hci0", name_prefix="AIRSEEKERS_", address=None, on_rx=None):
        import dbus
        import dbus.mainloop.glib
        from gi.repository import GLib
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.dbus, self.GLib = dbus, GLib
        self.ctx = GLib.MainContext.default()
        self.bus = dbus.SystemBus()
        self.adapter_path = "/org/bluez/" + adapter
        self.name_prefix, self.address = name_prefix, address.upper() if address else None
        self.on_rx = on_rx or (lambda data: None)
        self.dev_path = self.pipe_path = None
        self.notify_fd = None
        self.notify_mode = "-"
        self.mtu = None
        self.flags = {}
        self.inflight = 0
        self.write_errors = []
        self._signal = None

    # -- discovery / connection ---------------------------------------------------------------
    def _objects(self):
        om = self.dbus.Interface(self.bus.get_object(BLUEZ, "/"), OM_IF)
        return om.GetManagedObjects()

    def _match(self, props):
        if self.address:
            return str(props.get("Address", "")).upper() == self.address
        return str(props.get("Name", props.get("Alias", ""))).startswith(self.name_prefix)

    def _find_device(self):
        for path, ifaces in self._objects().items():
            if DEV_IF in ifaces and path.startswith(self.adapter_path + "/") \
                    and self._match(ifaces[DEV_IF]):
                return str(path), str(ifaces[DEV_IF].get("Name", ""))
        return None, None

    def find(self, timeout=20.0):
        path, name = self._find_device()
        if path:
            self.dev_path = path
            return name
        adapter = self.dbus.Interface(self.bus.get_object(BLUEZ, self.adapter_path),
                                      "org.bluez.Adapter1")
        try:
            adapter.SetDiscoveryFilter({"Transport": "le"})
        except self.dbus.DBusException:
            pass
        adapter.StartDiscovery()
        try:
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                self.pump(0.5)
                path, name = self._find_device()
                if path:
                    self.dev_path = path
                    return name
        finally:
            try:
                adapter.StopDiscovery()
            except self.dbus.DBusException:
                pass
        raise RuntimeError("no device %s found in %.0f s" %
                           (self.address or self.name_prefix + "*", timeout))

    def _dev_prop(self, name):
        props = self.dbus.Interface(self.bus.get_object(BLUEZ, self.dev_path), PROPS_IF)
        try:
            return props.Get(DEV_IF, name)
        except self.dbus.DBusException:
            return None

    def _wait_prop(self, name, value, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if bool(self._dev_prop(name)) == value:
                return True
            self.pump(0.02)
        return False

    def wait_advert(self, timeout=30.0):
        """Scan until the device's advertising is seen again. Seconds, or None on timeout."""
        seen = []
        sig = self.bus.add_signal_receiver(
            lambda iface, changed, inv: seen.append(1) if "RSSI" in changed else None,
            signal_name="PropertiesChanged", dbus_interface=PROPS_IF, path=self.dev_path,
            arg0=DEV_IF)
        adapter = self.dbus.Interface(self.bus.get_object(BLUEZ, self.adapter_path),
                                      "org.bluez.Adapter1")
        t0 = time.monotonic()
        try:
            try:
                adapter.SetDiscoveryFilter({"Transport": "le", "DuplicateData": True})
            except self.dbus.DBusException:
                pass
            adapter.StartDiscovery()
            while not seen and time.monotonic() - t0 < timeout:
                self.pump(0.01)
        finally:
            sig.remove()
            try:
                adapter.StopDiscovery()
            except self.dbus.DBusException:
                pass
        return time.monotonic() - t0 if seen else None

    def connect(self, timeout=30.0, attempts=3):
        """Returns (seconds in the successful Connect(), seconds to ServicesResolved from the
        start of that Connect(), number of Connect() calls). ``le-connection-abort-by-local``
        is retried: BlueZ often needs a second Connect()."""
        dev = self.dbus.Interface(self.bus.get_object(BLUEZ, self.dev_path), DEV_IF)
        for attempt in range(1, attempts + 1):
            t0 = time.monotonic()
            try:
                dev.Connect(timeout=timeout)
                break
            except self.dbus.DBusException as exc:
                if "AlreadyConnected" in exc.get_dbus_name():
                    break
                if "abort-by-local" not in str(exc) or attempt == attempts:
                    raise
                self.pump(0.5)
        t_conn = time.monotonic() - t0
        if not self._wait_prop("ServicesResolved", True, timeout):
            raise RuntimeError("services not resolved after %.0f s" % timeout)
        t_res = time.monotonic() - t0
        self._find_chars()
        self._start_notify()
        return t_conn, t_res, attempt

    def disconnect(self, timeout=10.0):
        self._stop_notify()
        dev = self.dbus.Interface(self.bus.get_object(BLUEZ, self.dev_path), DEV_IF)
        t0 = time.monotonic()
        try:
            dev.Disconnect(timeout=timeout)
        except self.dbus.DBusException:
            pass
        self._wait_prop("Connected", False, timeout)
        return time.monotonic() - t0

    def _find_chars(self):
        self.chars = {}
        for path, ifaces in self._objects().items():
            if CHAR_IF in ifaces and str(path).startswith(self.dev_path + "/"):
                props = ifaces[CHAR_IF]
                uuid = str(props["UUID"]).lower()
                self.chars[uuid] = str(path)
                self.flags[uuid] = [str(f) for f in props.get("Flags", [])]
                if uuid == PIPE_UUID and "MTU" in props:
                    self.mtu = int(props["MTU"])
        if PIPE_UUID not in self.chars:
            raise RuntimeError("serial pipe characteristic %s not found" % PIPE_UUID)
        self.pipe_path = self.chars[PIPE_UUID]
        self.pipe = self._char(PIPE_UUID)
        if WRITE_UUID not in self.chars:
            raise RuntimeError("write characteristic %s not found" % WRITE_UUID)
        self.tx = self._char(WRITE_UUID)

    def _char(self, uuid):
        return self.dbus.Interface(self.bus.get_object(BLUEZ, self.chars[uuid]), CHAR_IF)

    # -- notifications --------------------------------------------------------------------------
    def _start_notify(self):
        pipe = self.pipe
        try:
            fd, mtu = pipe.AcquireNotify(self.dbus.Dictionary({}, signature="sv"))
            self.notify_fd = fd.take()
            os.set_blocking(self.notify_fd, False)
            self.mtu = int(mtu)
            self.notify_mode = "AcquireNotify"
            return
        except self.dbus.DBusException as exc:
            self.notify_mode = "StartNotify (AcquireNotify: %s)" % exc.get_dbus_name()
        self._signal = self.bus.add_signal_receiver(
            self._on_props, signal_name="PropertiesChanged", dbus_interface=PROPS_IF,
            path=self.pipe_path, arg0=CHAR_IF)
        pipe.StartNotify()

    def _on_props(self, iface, changed, invalidated):
        if "Value" in changed:
            self.on_rx(bytes(bytearray(changed["Value"])))

    def _stop_notify(self):
        if self.notify_fd is not None:
            os.close(self.notify_fd)
            self.notify_fd = None
        if self._signal is not None:
            self._signal.remove()
            self._signal = None
            try:
                self._char(PIPE_UUID).StopNotify()
            except self.dbus.DBusException:
                pass

    # -- I/O ---------------------------------------------------------------------------------------
    def pump(self, seconds):
        """Run the D-Bus main loop and drain notifications for ``seconds`` (0 = one pass)."""
        end = time.monotonic() + seconds
        while True:
            while self.ctx.pending():
                self.ctx.iteration(False)
            if self.notify_fd is not None:
                while True:
                    try:
                        data = os.read(self.notify_fd, 4096)
                    except BlockingIOError:
                        break
                    if not data:
                        break
                    self.on_rx(data)
            if time.monotonic() >= end:
                return
            time.sleep(0.0005)

    def can_write_without_response(self):
        return "write-without-response" in self.flags.get(WRITE_UUID, [])

    def write(self, data, without_response=False, timeout=10.0):
        """Issue one GATT write; ``inflight`` drops when BlueZ answers. A write with response
        first waits for the previous write's reply: BlueZ rejects a second outstanding
        request with org.bluez.Error.InProgress."""
        if not without_response:
            end = time.monotonic() + timeout
            while self.inflight and time.monotonic() < end:
                self.pump(0)
        opts = self.dbus.Dictionary({"type": "command" if without_response else "request"},
                                    signature="sv")
        self.inflight += 1

        def ok():
            self.inflight -= 1

        def err(exc):
            self.inflight -= 1
            self.write_errors.append(str(exc.get_dbus_name()) + ": " + str(exc))

        self.tx.WriteValue(self.dbus.Array(bytearray(data), signature="y"), opts,
                           reply_handler=ok, error_handler=err, timeout=timeout)

    def read(self, uuid):
        return bytes(bytearray(self._char(uuid).ReadValue(
            self.dbus.Dictionary({}, signature="sv"))))

    def conn_interval(self):
        """Adapter's LE connection interval limits from debugfs, as text."""
        base = "/sys/kernel/debug/bluetooth/" + self.adapter_path.rsplit("/", 1)[1]
        out = []
        for name in ("conn_min_interval", "conn_max_interval"):
            try:
                with open(os.path.join(base, name)) as f:
                    v = int(f.read().strip())
                out.append("%s %d (%.2f ms)" % (name[5:8], v, v * 1.25))
            except (OSError, ValueError):
                return "n/a (needs root)"
        return ", ".join(out)

    def describe(self):
        return {"device": self._dev_prop("Name"), "mtu": self.mtu, "notify": self.notify_mode,
                "conn interval (adapter limits)": self.conn_interval(),
                "flags": {u[:8]: ",".join(f) for u, f in sorted(self.flags.items())}}

    def close(self):
        if self.dev_path:
            self.disconnect()
