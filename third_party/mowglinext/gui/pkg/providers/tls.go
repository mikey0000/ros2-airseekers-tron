package providers

import (
	"context"
	"crypto/ecdsa"
	"crypto/elliptic"
	"crypto/rand"
	"crypto/sha256"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"errors"
	"fmt"
	"log"
	"math/big"
	"net"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"sync"
	"time"
)

// The mower serves the GUI over HTTPS with a certificate from its own local
// CA, so a phone that installed that CA once gets a secure origin (needed by
// Web Bluetooth and service workers). The CA key never leaves the mower.

const (
	caValidity          = 10 * 365 * 24 * time.Hour
	serverCertValidity  = 2 * 365 * 24 * time.Hour
	serverCertRenewBy   = 30 * 24 * time.Hour
	tlsRefreshInterval  = 60 * time.Second
	caCertFile          = "ca.crt"
	caKeyFile           = "ca.key"
	backdateCertificate = time.Hour
)

// TLSDirFromEnv returns where the CA lives: TLS_DIR if set, else a "tls"
// directory inside the GUI DB directory (DB_PATH). The DB directory is the
// one path guaranteed to be on a persistent volume (/userdata/.../gui_db ->
// /db); bitcask ignores subdirectories, also when it merges.
func TLSDirFromEnv() string {
	if dir := os.Getenv("TLS_DIR"); dir != "" {
		return dir
	}
	return filepath.Join(os.Getenv("DB_PATH"), "tls")
}

// HostAddresses lists the addresses a client can reach this host on.
type HostAddresses func() ([]net.IP, error)

// TLSProvider owns the mower CA and the live server certificate.
type TLSProvider struct {
	dir       string
	hostname  func() (string, error)
	addresses HostAddresses
	now       func() time.Time

	caCert *x509.Certificate
	caKey  *ecdsa.PrivateKey
	caPEM  []byte

	mu         sync.RWMutex
	serverCert *tls.Certificate
	serverLeaf *x509.Certificate
	issuedFor  sanSet
}

// sanSet is what the server certificate is valid for.
type sanSet struct {
	Hostname string
	DNSNames []string
	IPs      []net.IP
}

func (s sanSet) key() string {
	parts := slices.Clone(s.DNSNames)
	for _, ip := range s.IPs {
		parts = append(parts, ip.String())
	}
	return strings.Join(parts, ",")
}

// NewTLSProvider loads (or creates) the CA in dir and issues a server
// certificate for the current host names and addresses.
func NewTLSProvider(dir string) (*TLSProvider, error) {
	return newTLSProvider(dir, os.Hostname, InterfaceAddresses, time.Now)
}

func newTLSProvider(dir string, hostname func() (string, error), addresses HostAddresses, now func() time.Time) (*TLSProvider, error) {
	p := &TLSProvider{dir: dir, hostname: hostname, addresses: addresses, now: now}
	if err := p.loadOrCreateCA(); err != nil {
		return nil, err
	}
	if _, err := p.Refresh(); err != nil {
		return nil, err
	}
	return p, nil
}

// Start re-checks the SAN set and the expiry every 60 s until ctx ends.
func (p *TLSProvider) Start(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(tlsRefreshInterval)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				if reissued, err := p.Refresh(); err != nil {
					log.Printf("tls: refreshing server certificate failed: %v", err)
				} else if reissued {
					log.Printf("tls: re-issued server certificate for %s", p.currentSANs().key())
				}
			}
		}
	}()
}

// Refresh re-issues the server certificate when the host names/addresses
// changed or it expires within 30 days. It reports whether it re-issued.
func (p *TLSProvider) Refresh() (bool, error) {
	want, err := p.wantedSANs()
	if err != nil {
		return false, err
	}
	p.mu.RLock()
	fresh := p.serverLeaf != nil &&
		p.issuedFor.key() == want.key() &&
		p.serverLeaf.NotAfter.Sub(p.now()) > serverCertRenewBy
	p.mu.RUnlock()
	if fresh {
		return false, nil
	}
	cert, leaf, err := p.issueServerCert(want)
	if err != nil {
		return false, err
	}
	p.mu.Lock()
	p.serverCert, p.serverLeaf, p.issuedFor = cert, leaf, want
	p.mu.Unlock()
	return true, nil
}

// GetCertificate serves the current server certificate (tls.Config hook).
func (p *TLSProvider) GetCertificate(*tls.ClientHelloInfo) (*tls.Certificate, error) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.serverCert == nil {
		return nil, errors.New("no server certificate")
	}
	return p.serverCert, nil
}

// TLSConfig is a server config that always presents the live certificate.
func (p *TLSProvider) TLSConfig() *tls.Config {
	return &tls.Config{MinVersion: tls.VersionTLS12, GetCertificate: p.GetCertificate}
}

// CAPEM is the CA certificate, PEM encoded, for clients to install.
func (p *TLSProvider) CAPEM() []byte { return slices.Clone(p.caPEM) }

// CAFingerprintSHA256 is the SHA-256 of the CA certificate DER as
// colon-separated upper-case hex, the form phones show when installing it.
func (p *TLSProvider) CAFingerprintSHA256() string {
	sum := sha256.Sum256(p.caCert.Raw)
	hexes := make([]string, len(sum))
	for i, b := range sum {
		hexes[i] = fmt.Sprintf("%02X", b)
	}
	return strings.Join(hexes, ":")
}

// CANotAfter is when the CA expires.
func (p *TLSProvider) CANotAfter() time.Time { return p.caCert.NotAfter }

// Hostname is the host name the current server certificate was issued for.
func (p *TLSProvider) Hostname() string { return p.currentSANs().Hostname }

// IPs are the addresses the current server certificate covers, loopback
// excluded, IPv4 first.
func (p *TLSProvider) IPs() []net.IP {
	var out []net.IP
	for _, ip := range p.currentSANs().IPs {
		if !ip.IsLoopback() {
			out = append(out, ip)
		}
	}
	return out
}

func (p *TLSProvider) currentSANs() sanSet {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.issuedFor
}

func (p *TLSProvider) wantedSANs() (sanSet, error) {
	host, err := p.hostname()
	if err != nil {
		return sanSet{}, fmt.Errorf("hostname: %w", err)
	}
	host = strings.ToLower(strings.TrimSuffix(host, "."))
	addrs, err := p.addresses()
	if err != nil {
		return sanSet{}, fmt.Errorf("interface addresses: %w", err)
	}

	var dns []string
	for _, name := range []string{host, host + ".local", "localhost"} {
		if name != "" && name != ".local" && p.dnsPermitted(name) && !slices.Contains(dns, name) {
			dns = append(dns, name)
		}
	}
	slices.Sort(dns)

	ips := []net.IP{net.ParseIP("127.0.0.1").To4()}
	for _, ip := range addrs {
		if v4 := ip.To4(); v4 != nil {
			ip = v4
		}
		if !slices.ContainsFunc(ips, ip.Equal) {
			ips = append(ips, ip)
		}
	}
	slices.SortFunc(ips, compareIPs)
	return sanSet{Hostname: host, DNSNames: dns, IPs: ips}, nil
}

// compareIPs orders IPv4 before IPv6, then bytewise.
func compareIPs(a, b net.IP) int {
	a4, b4 := a.To4() != nil, b.To4() != nil
	if a4 != b4 {
		if a4 {
			return -1
		}
		return 1
	}
	return slices.Compare(a, b)
}

// dnsPermitted mirrors the CA's name constraints so a host rename never
// produces a leaf the CA is not allowed to vouch for.
func (p *TLSProvider) dnsPermitted(name string) bool {
	for _, c := range p.caCert.PermittedDNSDomains {
		if name == c || strings.HasSuffix(name, "."+c) {
			return true
		}
	}
	return false
}

// InterfaceAddresses returns every up, non-loopback unicast IPv4 address and
// every global IPv6 address, skipping container bridges (docker0, br-*,
// veth*) that a phone can never reach.
func InterfaceAddresses() ([]net.IP, error) {
	ifaces, err := net.Interfaces()
	if err != nil {
		return nil, err
	}
	var out []net.IP
	for _, iface := range ifaces {
		if iface.Flags&net.FlagUp == 0 || iface.Flags&net.FlagLoopback != 0 || isContainerInterface(iface.Name) {
			continue
		}
		addrs, err := iface.Addrs()
		if err != nil {
			continue
		}
		for _, a := range addrs {
			ipNet, ok := a.(*net.IPNet)
			if !ok {
				continue
			}
			if usableAddress(ipNet.IP) {
				out = append(out, ipNet.IP)
			}
		}
	}
	return out, nil
}

func isContainerInterface(name string) bool {
	return strings.HasPrefix(name, "docker") || strings.HasPrefix(name, "br-") || strings.HasPrefix(name, "veth")
}

func usableAddress(ip net.IP) bool {
	if ip.IsLoopback() || ip.IsLinkLocalUnicast() || ip.IsMulticast() || ip.IsUnspecified() {
		return false
	}
	return ip.IsGlobalUnicast()
}

func (p *TLSProvider) loadOrCreateCA() error {
	if err := os.MkdirAll(p.dir, 0o700); err != nil {
		return fmt.Errorf("creating %s: %w", p.dir, err)
	}
	certPath, keyPath := filepath.Join(p.dir, caCertFile), filepath.Join(p.dir, caKeyFile)
	certPEM, certErr := os.ReadFile(certPath)
	keyPEM, keyErr := os.ReadFile(keyPath)
	if certErr == nil && keyErr == nil {
		cert, key, err := parseCA(certPEM, keyPEM)
		if err == nil {
			p.caCert, p.caKey, p.caPEM = cert, key, certPEM
			return nil
		}
		log.Printf("tls: stored CA in %s is unusable (%v); moving it aside and creating a new one", p.dir, err)
	} else if certErr == nil || keyErr == nil {
		log.Printf("tls: only one of %s/%s exists in %s; moving it aside and creating a new CA", caCertFile, caKeyFile, p.dir)
	}
	if certErr == nil || keyErr == nil {
		suffix := ".bad-" + p.now().UTC().Format("20060102T150405Z")
		_ = os.Rename(certPath, certPath+suffix)
		_ = os.Rename(keyPath, keyPath+suffix)
	}
	return p.createCA(certPath, keyPath)
}

func parseCA(certPEM, keyPEM []byte) (*x509.Certificate, *ecdsa.PrivateKey, error) {
	certBlock, _ := pem.Decode(certPEM)
	if certBlock == nil || certBlock.Type != "CERTIFICATE" {
		return nil, nil, errors.New("ca.crt is not a PEM certificate")
	}
	cert, err := x509.ParseCertificate(certBlock.Bytes)
	if err != nil {
		return nil, nil, err
	}
	keyBlock, _ := pem.Decode(keyPEM)
	if keyBlock == nil || keyBlock.Type != "EC PRIVATE KEY" {
		return nil, nil, errors.New("ca.key is not a PEM EC private key")
	}
	key, err := x509.ParseECPrivateKey(keyBlock.Bytes)
	if err != nil {
		return nil, nil, err
	}
	if !key.PublicKey.Equal(cert.PublicKey) {
		return nil, nil, errors.New("ca.key does not match ca.crt")
	}
	if !cert.IsCA {
		return nil, nil, errors.New("ca.crt is not a CA certificate")
	}
	return cert, key, nil
}

func (p *TLSProvider) createCA(certPath, keyPath string) error {
	host, err := p.hostname()
	if err != nil {
		return fmt.Errorf("hostname: %w", err)
	}
	host = strings.ToLower(strings.TrimSuffix(host, "."))
	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		return err
	}
	serial, err := randomSerial()
	if err != nil {
		return err
	}
	now := p.now()
	// Name constraints: a phone that trusts this CA trusts it only for the
	// mower's own names, never for an arbitrary website, should the key
	// ever leak. IP SANs are left unconstrained (LAN addresses change).
	permitted := []string{"local", "localhost"}
	if host != "" && host != "localhost" && !strings.HasSuffix(host, ".local") {
		permitted = append(permitted, host)
	}
	template := &x509.Certificate{
		SerialNumber:                serial,
		Subject:                     pkix.Name{CommonName: strings.TrimSpace(host + " mower CA"), Organization: []string{"Mower GUI"}},
		NotBefore:                   now.Add(-backdateCertificate),
		NotAfter:                    now.Add(caValidity),
		KeyUsage:                    x509.KeyUsageCertSign | x509.KeyUsageCRLSign | x509.KeyUsageDigitalSignature,
		BasicConstraintsValid:       true,
		IsCA:                        true,
		MaxPathLenZero:              true,
		PermittedDNSDomainsCritical: true,
		PermittedDNSDomains:         permitted,
	}
	der, err := x509.CreateCertificate(rand.Reader, template, template, &key.PublicKey, key)
	if err != nil {
		return err
	}
	cert, err := x509.ParseCertificate(der)
	if err != nil {
		return err
	}
	keyDER, err := x509.MarshalECPrivateKey(key)
	if err != nil {
		return err
	}
	certPEM := pem.EncodeToMemory(&pem.Block{Type: "CERTIFICATE", Bytes: der})
	keyPEM := pem.EncodeToMemory(&pem.Block{Type: "EC PRIVATE KEY", Bytes: keyDER})
	// Key first: a crash between the two writes leaves a lone key, which the
	// next start moves aside, instead of a certificate without its key.
	if err := writeFileAtomic(keyPath, keyPEM, 0o600); err != nil {
		return err
	}
	if err := writeFileAtomic(certPath, certPEM, 0o644); err != nil {
		return err
	}
	log.Printf("tls: created mower CA %q in %s", cert.Subject.CommonName, p.dir)
	p.caCert, p.caKey, p.caPEM = cert, key, certPEM
	return nil
}

func (p *TLSProvider) issueServerCert(sans sanSet) (*tls.Certificate, *x509.Certificate, error) {
	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		return nil, nil, err
	}
	serial, err := randomSerial()
	if err != nil {
		return nil, nil, err
	}
	now := p.now()
	cn := sans.Hostname + ".local"
	if sans.Hostname == "" {
		cn = "localhost"
	}
	template := &x509.Certificate{
		SerialNumber: serial,
		Subject:      pkix.Name{CommonName: cn},
		NotBefore:    now.Add(-backdateCertificate),
		NotAfter:     now.Add(serverCertValidity),
		KeyUsage:     x509.KeyUsageDigitalSignature,
		ExtKeyUsage:  []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth},
		DNSNames:     sans.DNSNames,
		IPAddresses:  sans.IPs,
	}
	if template.NotAfter.After(p.caCert.NotAfter) {
		template.NotAfter = p.caCert.NotAfter
	}
	der, err := x509.CreateCertificate(rand.Reader, template, p.caCert, &key.PublicKey, p.caKey)
	if err != nil {
		return nil, nil, err
	}
	leaf, err := x509.ParseCertificate(der)
	if err != nil {
		return nil, nil, err
	}
	return &tls.Certificate{
		Certificate: [][]byte{der, p.caCert.Raw},
		PrivateKey:  key,
		Leaf:        leaf,
	}, leaf, nil
}

func randomSerial() (*big.Int, error) {
	return rand.Int(rand.Reader, new(big.Int).Lsh(big.NewInt(1), 127))
}

func writeFileAtomic(path string, data []byte, mode os.FileMode) error {
	tmp, err := os.CreateTemp(filepath.Dir(path), filepath.Base(path)+".tmp-*")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	defer os.Remove(tmpName)
	if err := tmp.Chmod(mode); err != nil {
		tmp.Close()
		return err
	}
	if _, err := tmp.Write(data); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Sync(); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	return os.Rename(tmpName, path)
}
