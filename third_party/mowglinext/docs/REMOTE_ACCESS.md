# Remote access

Upstream MowgliNext documents its Tailscale sidecar in this file. That part is
not vendored here; the code is `gui/pkg/providers/remote_access*.go`,
`gui/pkg/api/remote_access.go` and `gui/web/src/components/settings/RemoteAccess*`.
This copy holds the section the Tron port adds.

## Secure origin (HTTPS on 4443)

Web Bluetooth and service workers only run on a secure origin, and
`http://<mower>:4006` is not one. So the GUI backend also serves the same API
and web app over HTTPS on port 4443, with a certificate from a CA that the
mower creates for itself.

**How it works**

- On first start the backend creates a CA (ECDSA P-256, valid 10 years,
  CN `<hostname> mower CA`). The CA carries name constraints: it can only vouch
  for `<hostname>`, `*.local` and `localhost` (IP addresses are not
  constrained). A leaked CA key therefore cannot impersonate an ordinary website
  on a phone that trusts it.
- From that CA it issues a server certificate for `<hostname>`,
  `<hostname>.local`, `localhost`, `127.0.0.1` and every non-loopback IPv4 and
  global IPv6 address of the host (container bridges `docker*`, `br-*`, `veth*`
  are skipped). The container runs with `network_mode: host`, so these are the
  mower's LAN addresses. The server certificate is valid for 2 years and kept in
  memory only. Every 60 s the backend re-checks the names and addresses and
  re-issues the certificate when they changed or when fewer than 30 days remain.
  Clients that trust the CA do not notice.
- The listener address is the DB key `system.api.tls_addr` (env fallback
  `API_TLS_ADDR`, default `:4443`). If the CA or certificate cannot be set up,
  or the port is taken, the backend logs it and keeps serving plain HTTP on 4006.

**Where the CA lives**

`ca.key` (mode 0600) and `ca.crt` (0644) are stored in `$TLS_DIR`, by default
the `tls/` subdirectory of the GUI database directory (`$DB_PATH/tls`). On the
Tron compose setup that is `/db/tls` in the container, which is on the
persistent `gui_db` volume, so the CA survives image updates and phones stay
trusted. bitcask ignores subdirectories, including when it merges its files. If
only one of the two files is present, or they do not parse or match, they are
renamed to `*.bad-<timestamp>` and a new CA is created. Every phone then has to
install the new CA.

**Installing the CA**

Settings → Remote access → "Secure origin (HTTPS)" shows whether the current page
is already on a secure origin, lists the HTTPS addresses, offers the CA download
(`GET /api/secure-origin/ca.crt`, saved as `mower-ca.crt`) and shows its SHA-256
fingerprint. Compare the fingerprint with what the phone shows before trusting
the certificate. Once per device:

- Android: Settings → Security & privacy → More security settings → Encryption
  & credentials → Install a certificate → CA certificate, then pick
  `mower-ca.crt` (wording varies by vendor). Then open
  `https://<hostname>.local:4443` in Chrome.
- Chrome on a computer: `chrome://settings/certificates` → Authorities → Import,
  and tick "Trust this certificate for identifying websites".
- iOS: install the downloaded profile, then enable full trust under Settings →
  General → About → Certificate Trust Settings. This gives a secure origin, but
  Web Bluetooth is still unavailable on iOS.

`GET /api/secure-origin` returns `{enabled, https_urls, hostname,
ca_fingerprint_sha256, ca_not_after}`, which the card displays.

**Not implemented yet**

- Factory reset does not regenerate the CA. To replace it by hand, stop the GUI
  container, delete `$TLS_DIR/ca.key` and `$TLS_DIR/ca.crt`, and start it again.
- `<hostname>.local` only resolves if the host announces itself over mDNS
  (avahi). Otherwise use the IP address URL.
- There is no warning before the CA expires (after 10 years).
