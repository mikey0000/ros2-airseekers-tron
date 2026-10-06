package api

type OkResponse struct {
	Ok string `json:"ok,omitempty"`
}
type ErrorResponse struct {
	Error string `json:"error,omitempty"`
}

type SettingsStatusResponse struct {
	OnboardingCompleted bool `json:"onboarding_completed"`
}

type GetSettingsResponse struct {
	Settings map[string]any `json:"settings,omitempty"`
}

type GetConfigResponse struct {
	TileUri string `json:"tileUri"`
}

type Container struct {
	ID     string            `json:"id"`
	Names  []string          `json:"names"`
	Labels map[string]string `json:"labels"`
	State  string            `json:"state"`
}

type ContainerListResponse struct {
	// Available is false when this host has no reachable Docker daemon (the
	// stack runs natively); Containers is then empty. Not an error: the Logs
	// page falls back to the /rosout stream.
	Available  bool        `json:"available"`
	Containers []Container `json:"containers"`
}
