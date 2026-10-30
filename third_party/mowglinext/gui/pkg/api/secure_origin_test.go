package api

import (
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type fakeSecureOrigin struct{}

func (fakeSecureOrigin) Hostname() string { return "test-mower" }
func (fakeSecureOrigin) IPs() []net.IP {
	return []net.IP{net.ParseIP("192.0.2.10"), net.ParseIP("2001:db8::10")}
}
func (fakeSecureOrigin) CAPEM() []byte {
	return []byte("-----BEGIN CERTIFICATE-----\nAAAA\n-----END CERTIFICATE-----\n")
}
func (fakeSecureOrigin) CAFingerprintSHA256() string { return "AA:BB" }
func (fakeSecureOrigin) CANotAfter() time.Time {
	return time.Date(2036, 10, 27, 12, 0, 0, 0, time.FixedZone("x", 3600))
}

func serveSecureOrigin(source SecureOriginSource, path string) *httptest.ResponseRecorder {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	SecureOriginRoutes(r.Group("/api"), source, ":4443")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(http.MethodGet, path, nil))
	return w
}

func TestSecureOriginStatus(t *testing.T) {
	w := serveSecureOrigin(fakeSecureOrigin{}, "/api/secure-origin")
	require.Equal(t, http.StatusOK, w.Code)
	var body SecureOriginResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
	assert.True(t, body.Enabled)
	assert.Equal(t, "test-mower", body.Hostname)
	assert.Equal(t, []string{
		"https://test-mower.local:4443",
		"https://192.0.2.10:4443",
		"https://[2001:db8::10]:4443",
	}, body.HTTPSURLs)
	assert.Equal(t, "AA:BB", body.CAFingerprintSHA256)
	assert.Equal(t, "2036-10-27T11:00:00Z", body.CANotAfter)
}

func TestSecureOriginDisabledWithoutCertificate(t *testing.T) {
	w := serveSecureOrigin(nil, "/api/secure-origin")
	require.Equal(t, http.StatusOK, w.Code)
	var raw map[string]any
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &raw))
	assert.Equal(t, false, raw["enabled"])
	assert.Equal(t, []any{}, raw["https_urls"])

	assert.Equal(t, http.StatusNotFound, serveSecureOrigin(nil, "/api/secure-origin/ca.crt").Code)
}

func TestSecureOriginCADownload(t *testing.T) {
	w := serveSecureOrigin(fakeSecureOrigin{}, "/api/secure-origin/ca.crt")
	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, "application/x-x509-ca-cert", w.Header().Get("Content-Type"))
	assert.Equal(t, `attachment; filename="mower-ca.crt"`, w.Header().Get("Content-Disposition"))
	assert.Contains(t, w.Body.String(), "BEGIN CERTIFICATE")
}

func TestSecureOriginURLsDefaultPort(t *testing.T) {
	assert.Equal(t, []string{"https://m.local"}, secureOriginURLs("m", nil, ":443"))
	assert.Equal(t, []string{"https://m.local:4443"}, secureOriginURLs("m", nil, "bogus"))
}
