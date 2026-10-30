package providers

import (
	"crypto/tls"
	"crypto/x509"
	"io"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

type fakeHost struct {
	name  string
	addrs []net.IP
	now   time.Time
}

func (f *fakeHost) hostname() (string, error)    { return f.name, nil }
func (f *fakeHost) addresses() ([]net.IP, error) { return f.addrs, nil }
func (f *fakeHost) clock() time.Time             { return f.now }

func newFakeHost() *fakeHost {
	return &fakeHost{
		name:  "Test-Mower",
		addrs: []net.IP{net.ParseIP("192.0.2.10"), net.ParseIP("2001:db8::10")},
		now:   time.Date(2026, 10, 30, 12, 0, 0, 0, time.UTC),
	}
}

func newTestTLSProvider(t *testing.T, dir string, h *fakeHost) *TLSProvider {
	t.Helper()
	p, err := newTLSProvider(dir, h.hostname, h.addresses, h.clock)
	if err != nil {
		t.Fatalf("newTLSProvider: %v", err)
	}
	return p
}

func currentLeaf(t *testing.T, p *TLSProvider) *x509.Certificate {
	t.Helper()
	cert, err := p.GetCertificate(nil)
	if err != nil {
		t.Fatal(err)
	}
	return cert.Leaf
}

func TestTLSProviderCreatesAndPersistsCA(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "tls")
	h := newFakeHost()
	p := newTestTLSProvider(t, dir, h)

	keyInfo, err := os.Stat(filepath.Join(dir, "ca.key"))
	if err != nil {
		t.Fatal(err)
	}
	if keyInfo.Mode().Perm() != 0o600 {
		t.Errorf("ca.key mode = %v, want 0600", keyInfo.Mode().Perm())
	}
	certInfo, err := os.Stat(filepath.Join(dir, "ca.crt"))
	if err != nil {
		t.Fatal(err)
	}
	if certInfo.Mode().Perm() != 0o644 {
		t.Errorf("ca.crt mode = %v, want 0644", certInfo.Mode().Perm())
	}
	if !p.caCert.IsCA || p.caCert.Subject.CommonName != "test-mower mower CA" {
		t.Errorf("unexpected CA: IsCA=%v CN=%q", p.caCert.IsCA, p.caCert.Subject.CommonName)
	}
	if got := p.CANotAfter().Sub(h.now); got < 9*365*24*time.Hour {
		t.Errorf("CA validity %v, want ~10 years", got)
	}
	fp := p.CAFingerprintSHA256()
	if len(fp) != 32*3-1 || strings.ToUpper(fp) != fp {
		t.Errorf("fingerprint %q is not upper-case colon hex", fp)
	}

	// A restart reuses the stored CA instead of minting a new one.
	again := newTestTLSProvider(t, dir, h)
	if again.CAFingerprintSHA256() != fp {
		t.Error("restart created a new CA")
	}
}

func TestTLSProviderServerCertCoversHostNamesAndAddresses(t *testing.T) {
	h := newFakeHost()
	p := newTestTLSProvider(t, t.TempDir(), h)
	leaf := currentLeaf(t, p)

	wantDNS := []string{"localhost", "test-mower", "test-mower.local"}
	if strings.Join(leaf.DNSNames, ",") != strings.Join(wantDNS, ",") {
		t.Errorf("DNSNames = %v, want %v", leaf.DNSNames, wantDNS)
	}
	var ips []string
	for _, ip := range leaf.IPAddresses {
		ips = append(ips, ip.String())
	}
	if strings.Join(ips, ",") != "127.0.0.1,192.0.2.10,2001:db8::10" {
		t.Errorf("IPAddresses = %v", ips)
	}
	if got := leaf.NotAfter.Sub(h.now); got < 729*24*time.Hour || got > 731*24*time.Hour {
		t.Errorf("leaf validity %v, want 2 years", got)
	}

	roots := x509.NewCertPool()
	roots.AddCert(p.caCert)
	for _, name := range []string{"test-mower.local", "192.0.2.10", "2001:db8::10", "localhost"} {
		if _, err := leaf.Verify(x509.VerifyOptions{DNSName: name, Roots: roots, CurrentTime: h.now}); err != nil {
			t.Errorf("leaf does not verify for %s: %v", name, err)
		}
	}
	if p.Hostname() != "test-mower" {
		t.Errorf("Hostname = %q", p.Hostname())
	}
	if len(p.IPs()) != 2 {
		t.Errorf("IPs() = %v, want the two non-loopback addresses", p.IPs())
	}
}

func TestTLSProviderCANameConstraintsRejectForeignNames(t *testing.T) {
	h := newFakeHost()
	p := newTestTLSProvider(t, t.TempDir(), h)
	// Even a certificate signed by the CA must not validate for another site.
	evil, _, err := p.issueServerCert(sanSet{Hostname: "x", DNSNames: []string{"bank.example"}})
	if err != nil {
		t.Fatal(err)
	}
	roots := x509.NewCertPool()
	roots.AddCert(p.caCert)
	if _, err := evil.Leaf.Verify(x509.VerifyOptions{DNSName: "bank.example", Roots: roots, CurrentTime: h.now}); err == nil {
		t.Fatal("CA without name constraints: a leaked key could impersonate any site")
	}
}

func TestTLSProviderReissuesOnAddressChangeAndExpiry(t *testing.T) {
	h := newFakeHost()
	p := newTestTLSProvider(t, t.TempDir(), h)
	first := currentLeaf(t, p)

	if reissued, err := p.Refresh(); err != nil || reissued {
		t.Fatalf("unchanged host: reissued=%v err=%v", reissued, err)
	}

	h.addrs = []net.IP{net.ParseIP("198.51.100.7")}
	if reissued, err := p.Refresh(); err != nil || !reissued {
		t.Fatalf("address change: reissued=%v err=%v", reissued, err)
	}
	second := currentLeaf(t, p)
	if second.SerialNumber.Cmp(first.SerialNumber) == 0 {
		t.Fatal("GetCertificate still serves the old certificate")
	}
	if second.IPAddresses[1].String() != "198.51.100.7" {
		t.Errorf("new leaf IPs = %v", second.IPAddresses)
	}

	h.now = second.NotAfter.Add(-29 * 24 * time.Hour)
	if reissued, err := p.Refresh(); err != nil || !reissued {
		t.Fatalf("near expiry: reissued=%v err=%v", reissued, err)
	}
}

func TestTLSProviderReplacesUnusableCA(t *testing.T) {
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, "ca.crt"), []byte("garbage"), 0o644); err != nil {
		t.Fatal(err)
	}
	p := newTestTLSProvider(t, dir, newFakeHost())
	if p.caCert == nil {
		t.Fatal("no CA")
	}
	matches, _ := filepath.Glob(filepath.Join(dir, "ca.crt.bad-*"))
	if len(matches) != 1 {
		t.Errorf("unusable ca.crt not moved aside: %v", matches)
	}
}

func TestTLSProviderServesHTTPS(t *testing.T) {
	h := newFakeHost()
	h.now = time.Now()
	h.addrs = nil
	p := newTestTLSProvider(t, t.TempDir(), h)

	// The same wiring as api.go: a plain http.Server whose only certificate
	// source is GetCertificate (httptest.StartTLS would inject its own).
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	srv := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
			_, _ = io.WriteString(w, "ok")
		}),
		TLSConfig: p.TLSConfig(),
	}
	go func() { _ = srv.ServeTLS(ln, "", "") }()
	defer srv.Close()

	roots := x509.NewCertPool()
	if !roots.AppendCertsFromPEM(p.CAPEM()) {
		t.Fatal("CAPEM is not a PEM certificate")
	}
	client := &http.Client{Transport: &http.Transport{TLSClientConfig: &tls.Config{RootCAs: roots}}}
	resp, err := client.Get("https://" + ln.Addr().String())
	if err != nil {
		t.Fatalf("HTTPS request with the mower CA trusted failed: %v", err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if string(body) != "ok" {
		t.Errorf("body = %q", body)
	}
}

func TestTLSDirFromEnv(t *testing.T) {
	t.Setenv("DB_PATH", "/data/db")
	t.Setenv("TLS_DIR", "")
	if got := TLSDirFromEnv(); got != "/data/db/tls" {
		t.Errorf("TLSDirFromEnv = %q", got)
	}
	t.Setenv("TLS_DIR", "/elsewhere")
	if got := TLSDirFromEnv(); got != "/elsewhere" {
		t.Errorf("TLSDirFromEnv with override = %q", got)
	}
}
