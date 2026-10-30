package api

import (
	"net"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
)

// SecureOriginSource is what the secure-origin endpoints need from the TLS
// provider (providers.TLSProvider implements it).
type SecureOriginSource interface {
	Hostname() string
	IPs() []net.IP
	CAPEM() []byte
	CAFingerprintSHA256() string
	CANotAfter() time.Time
}

// SecureOriginResponse tells the GUI where the HTTPS listener is reachable
// and which CA a phone must trust for it.
type SecureOriginResponse struct {
	Enabled             bool     `json:"enabled"`
	HTTPSURLs           []string `json:"https_urls"`
	Hostname            string   `json:"hostname"`
	CAFingerprintSHA256 string   `json:"ca_fingerprint_sha256,omitempty"`
	CANotAfter          string   `json:"ca_not_after,omitempty"`
}

// SecureOriginRoutes registers the HTTPS / mower-CA endpoints. source is nil
// when the certificate setup failed and only HTTP is served.
func SecureOriginRoutes(r *gin.RouterGroup, source SecureOriginSource, tlsAddr string) {
	group := r.Group("/secure-origin")
	group.GET("", getSecureOrigin(source, tlsAddr))
	group.GET("/ca.crt", getSecureOriginCA(source))
}

// getSecureOrigin returns the HTTPS URLs and the mower CA fingerprint
//
// @Summary secure origin (HTTPS) status
// @Description Whether the HTTPS listener runs, the URLs it answers on and the fingerprint of the mower CA a phone must install to trust them.
// @Tags secure-origin
// @Produce json
// @Success 200 {object} SecureOriginResponse
// @Router /secure-origin [get]
func getSecureOrigin(source SecureOriginSource, tlsAddr string) gin.HandlerFunc {
	return func(c *gin.Context) {
		if source == nil {
			host, _ := os.Hostname()
			c.JSON(http.StatusOK, SecureOriginResponse{HTTPSURLs: []string{}, Hostname: strings.ToLower(host)})
			return
		}
		c.JSON(http.StatusOK, SecureOriginResponse{
			Enabled:             true,
			HTTPSURLs:           secureOriginURLs(source.Hostname(), source.IPs(), tlsAddr),
			Hostname:            source.Hostname(),
			CAFingerprintSHA256: source.CAFingerprintSHA256(),
			CANotAfter:          source.CANotAfter().UTC().Format(time.RFC3339),
		})
	}
}

// getSecureOriginCA downloads the mower CA certificate
//
// @Summary download the mower CA certificate
// @Description PEM certificate of the mower's own CA. Install it on a phone once to make the HTTPS URLs a secure origin.
// @Tags secure-origin
// @Produce application/x-x509-ca-cert
// @Success 200 {file} file
// @Failure 404 {object} ErrorResponse
// @Router /secure-origin/ca.crt [get]
func getSecureOriginCA(source SecureOriginSource) gin.HandlerFunc {
	return func(c *gin.Context) {
		if source == nil {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "HTTPS is not available: the mower certificate could not be set up"})
			return
		}
		c.Header("Content-Disposition", `attachment; filename="mower-ca.crt"`)
		c.Data(http.StatusOK, "application/x-x509-ca-cert", source.CAPEM())
	}
}

// secureOriginURLs lists the mDNS name first, then every address.
func secureOriginURLs(hostname string, ips []net.IP, tlsAddr string) []string {
	port := tlsPort(tlsAddr)
	urls := []string{}
	if hostname != "" {
		urls = append(urls, "https://"+withPort(hostname+".local", port))
	}
	for _, ip := range ips {
		urls = append(urls, "https://"+withPort(ip.String(), port))
	}
	return urls
}

func tlsPort(addr string) string {
	_, port, err := net.SplitHostPort(addr)
	if err != nil || port == "" {
		return "4443"
	}
	return port
}

func withPort(host, port string) string {
	if port == "443" {
		if strings.Contains(host, ":") {
			return "[" + host + "]"
		}
		return host
	}
	return net.JoinHostPort(host, port)
}
