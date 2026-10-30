package api

import (
	"crypto/rand"
	"encoding/base32"
	"fmt"
	"net/http"
	"strings"
	"sync"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// BleTokenDBKey is where the BLE tunnel token lives in the GUI DB. The BLE
// bridge (scripts/ble/ble_bridge.py) reads it through GET /api/ble/token on
// 127.0.0.1:4006; the key and the response shape are a cross-job contract.
const BleTokenDBKey = "ble.token"

// bleTokenBytes of randomness give a 32-character base32 token.
const bleTokenBytes = 20

var bleTokenEncoding = base32.StdEncoding.WithPadding(base32.NoPadding)

// BleTokenResponse carries the BLE tunnel token.
type BleTokenResponse struct {
	Token string `json:"token"`
}

// bleTokens serialises minting so two concurrent first reads agree.
type bleTokens struct {
	db types.IDBProvider
	mu sync.Mutex
}

// BleTokenRoutes registers the BLE tunnel token endpoints.
func BleTokenRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	tokens := &bleTokens{db: dbProvider}
	group := r.Group("/ble")
	group.GET("/token", getBleToken(tokens))
	group.POST("/token/rotate", postBleTokenRotate(tokens))
}

// getBleToken returns the BLE tunnel token, minting it on first read
//
// @Summary BLE tunnel token
// @Description The token a Bluetooth client must present in the tunnel hello. Minted on first read (20 random bytes, base32 without padding, 32 characters) and persisted under the DB key ble.token.
// @Tags ble
// @Produce json
// @Success 200 {object} BleTokenResponse
// @Failure 500 {object} ErrorResponse
// @Router /ble/token [get]
func getBleToken(tokens *bleTokens) gin.HandlerFunc {
	return func(c *gin.Context) {
		token, err := tokens.get()
		respondBleToken(c, token, err)
	}
}

// postBleTokenRotate replaces the BLE tunnel token
//
// @Summary rotate the BLE tunnel token
// @Description Mints a new token; Bluetooth clients holding the old one are refused from the next hello on.
// @Tags ble
// @Produce json
// @Success 200 {object} BleTokenResponse
// @Failure 500 {object} ErrorResponse
// @Router /ble/token/rotate [post]
func postBleTokenRotate(tokens *bleTokens) gin.HandlerFunc {
	return func(c *gin.Context) {
		token, err := tokens.rotate()
		respondBleToken(c, token, err)
	}
}

func respondBleToken(c *gin.Context, token string, err error) {
	c.Header("Cache-Control", "no-store")
	if err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, BleTokenResponse{Token: token})
}

func (b *bleTokens) get() (string, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	value, err := b.db.Get(BleTokenDBKey)
	if err == nil && len(value) > 0 {
		return string(value), nil
	}
	// Mint only when the key is genuinely absent: on any other DB error a
	// fresh token would silently lock out every phone that stored the old one.
	if err != nil && !strings.Contains(err.Error(), "not found") {
		return "", fmt.Errorf("reading %s: %w", BleTokenDBKey, err)
	}
	return b.mintLocked()
}

func (b *bleTokens) rotate() (string, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.mintLocked()
}

func (b *bleTokens) mintLocked() (string, error) {
	token, err := newBleToken()
	if err != nil {
		return "", err
	}
	if err := b.db.Set(BleTokenDBKey, []byte(token)); err != nil {
		return "", fmt.Errorf("storing %s: %w", BleTokenDBKey, err)
	}
	return token, nil
}

func newBleToken() (string, error) {
	raw := make([]byte, bleTokenBytes)
	if _, err := rand.Read(raw); err != nil {
		return "", err
	}
	return bleTokenEncoding.EncodeToString(raw), nil
}
