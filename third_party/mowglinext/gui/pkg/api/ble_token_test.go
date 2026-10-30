package api

import (
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"regexp"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

var bleTokenPattern = regexp.MustCompile(`^[A-Z2-7]{32}$`)

func setupBleTokenRouter(db types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	BleTokenRoutes(r.Group("/api"), db)
	return r
}

func bleTokenRequest(t *testing.T, r *gin.Engine, method, path string) (int, BleTokenResponse) {
	t.Helper()
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest(method, path, nil))
	var body BleTokenResponse
	if w.Code == http.StatusOK {
		require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
		assert.Equal(t, "no-store", w.Header().Get("Cache-Control"))
	}
	return w.Code, body
}

func TestBleTokenMintedOnFirstReadAndStable(t *testing.T) {
	db := types.NewMockDBProvider()
	r := setupBleTokenRouter(db)

	code, first := bleTokenRequest(t, r, http.MethodGet, "/api/ble/token")
	require.Equal(t, http.StatusOK, code)
	assert.Regexp(t, bleTokenPattern, first.Token)

	stored, err := db.Get("ble.token")
	require.NoError(t, err)
	assert.Equal(t, first.Token, string(stored))

	_, again := bleTokenRequest(t, r, http.MethodGet, "/api/ble/token")
	assert.Equal(t, first.Token, again.Token)
}

func TestBleTokenRotateReplacesToken(t *testing.T) {
	db := types.NewMockDBProvider()
	r := setupBleTokenRouter(db)

	_, before := bleTokenRequest(t, r, http.MethodGet, "/api/ble/token")
	code, rotated := bleTokenRequest(t, r, http.MethodPost, "/api/ble/token/rotate")
	require.Equal(t, http.StatusOK, code)
	assert.Regexp(t, bleTokenPattern, rotated.Token)
	assert.NotEqual(t, before.Token, rotated.Token)

	_, after := bleTokenRequest(t, r, http.MethodGet, "/api/ble/token")
	assert.Equal(t, rotated.Token, after.Token)
}

// brokenDB fails reads with something other than "not found".
type brokenDB struct{ *types.MockDBProvider }

func (brokenDB) Get(string) ([]byte, error) { return nil, errors.New("bitcask: checksum failed") }

func TestBleTokenNotRemintedOnDBError(t *testing.T) {
	db := brokenDB{types.NewMockDBProvider()}
	r := setupBleTokenRouter(db)

	code, _ := bleTokenRequest(t, r, http.MethodGet, "/api/ble/token")
	assert.Equal(t, http.StatusInternalServerError, code)
	_, err := db.MockDBProvider.Get("ble.token")
	assert.Error(t, err, "a read error must not overwrite the stored token")
}
