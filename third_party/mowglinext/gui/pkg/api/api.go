package api

import (
	"context"
	"log"
	"net/http"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/docs"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	swaggerfiles "github.com/swaggo/files"
	ginSwagger "github.com/swaggo/gin-swagger"
)

// gin-swagger middleware
// swagger embed files

func NewAPI(dbProvider types.IDBProvider, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, firmwareProvider *providers.FirmwareProvider, irriSenseProvider *providers.IrriSenseProvider, remoteAccessProvider *providers.RemoteAccessProvider, notificationProvider *providers.NotificationProvider) {
	httpAddr, err := dbProvider.Get("system.api.addr")
	if err != nil {
		log.Fatal(err)
	}

	gin.SetMode(gin.ReleaseMode)
	docs.SwaggerInfo.BasePath = "/api"
	r := gin.Default()
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true
	config.AllowWebSockets = true
	r.Use(cors.New(config))
	r.Use(updateMaintenanceMiddleware)
	webDirectory, err := dbProvider.Get("system.api.webDirectory")
	if err != nil {
		log.Fatal(err)
	}
	webDir := string(webDirectory)
	registerWebUI(r, webDir)
	apiGroup := r.Group("/api")
	ConfigRoute(apiGroup, dbProvider)
	SettingsRoutes(apiGroup, dbProvider)
	ParamBindingRoutes(apiGroup, dbProvider, rosProvider)
	GNSSRoutes(apiGroup, dbProvider, dockerProvider)
	ContainersRoutes(apiGroup, dockerProvider)
	RosoutRoutes(apiGroup, rosProvider)
	MowgliNextRoutes(apiGroup, rosProvider)
	ImageryRoutes(apiGroup)
	SetupRoutes(apiGroup, firmwareProvider)
	RobotProfileRoutes(apiGroup, dbProvider)
	SystemRoutes(apiGroup)
	VersionsRoutes(apiGroup, dockerProvider)
	UpdatesRoutes(apiGroup, dockerProvider)
	UpdaterRoutes(apiGroup, rosProvider)
	DiagnosticsRoutes(apiGroup, dockerProvider, rosProvider, dbProvider)
	RosbagRoutes(apiGroup, dockerProvider)
	WeatherRoutes(apiGroup, dbProvider)
	ParamsRoutes(apiGroup, rosProvider)
	NtripRoutes(apiGroup)
	CorrectionsRoutes(apiGroup, rosProvider)
	RtkBaseRoutes(apiGroup, rosProvider)
	CalibrationRoutes(apiGroup, rosProvider, dbProvider)
	DriveTuningRoutes(apiGroup, dbProvider, dockerProvider)
	ScheduleRoutes(apiGroup, dbProvider)
	IrriSenseRoutes(apiGroup, irriSenseProvider)
	RemoteAccessRoutes(apiGroup, remoteAccessProvider)
	NotificationRoutes(apiGroup, notificationProvider)
	ImportRoutes(apiGroup, rosProvider, dbProvider)
	CameraRoutes(apiGroup, dbProvider)
	BleTokenRoutes(apiGroup, dbProvider)
	tlsAddr, tlsProvider := setupTLS(dbProvider)
	if tlsProvider != nil {
		SecureOriginRoutes(apiGroup, tlsProvider, tlsAddr)
	} else {
		SecureOriginRoutes(apiGroup, nil, tlsAddr)
	}
	tileServer, err := dbProvider.Get("system.map.enabled")
	if err != nil {
		log.Fatal(err)
	}
	if string(tileServer) == "true" {
		TilesProxy(r, dbProvider)
	}
	r.GET("/swagger/*any", ginSwagger.WrapHandler(swaggerfiles.Handler))
	if tlsProvider != nil {
		go serveTLS(r, tlsAddr, tlsProvider)
	}
	r.Run(string(httpAddr))
}

// setupTLS prepares the mower CA and server certificate. Any failure only
// costs the HTTPS listener: it is logged and the GUI stays on plain HTTP.
func setupTLS(dbProvider types.IDBProvider) (string, *providers.TLSProvider) {
	addr, err := dbProvider.Get("system.api.tls_addr")
	if err != nil || len(addr) == 0 {
		log.Printf("tls: no system.api.tls_addr (%v); serving HTTP only", err)
		return "", nil
	}
	dir := providers.TLSDirFromEnv()
	p, err := providers.NewTLSProvider(dir)
	if err != nil {
		log.Printf("tls: certificate setup in %s failed, serving HTTP only: %v", dir, err)
		return string(addr), nil
	}
	p.Start(context.Background())
	return string(addr), p
}

// serveTLS serves the same gin engine over HTTPS. It never takes the
// process down: a bind failure leaves HTTP running.
func serveTLS(handler http.Handler, addr string, p *providers.TLSProvider) {
	srv := &http.Server{Addr: addr, Handler: handler, TLSConfig: p.TLSConfig()}
	log.Printf("tls: serving HTTPS on %s", addr)
	if err := srv.ListenAndServeTLS("", ""); err != nil {
		log.Printf("tls: HTTPS listener on %s stopped: %v", addr, err)
	}
}
