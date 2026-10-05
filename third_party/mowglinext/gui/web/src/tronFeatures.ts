// Airseekers Tron port: single feature-flag module for the vendored MowgliNext GUI.
//
// The Tron stack has no STM32 firmware to flash, no Mowgli GNSS sidecar, no
// drive-tuning container, no host updater and no Docker socket, so the screens
// that drive those are hidden (never deleted) here. Every Tron-specific edit in
// the upstream tree is a one-line `isTronHidden(...)` check against this list,
// which keeps a future rebase onto a newer upstream commit trivial.
//
// Build-time switches (vite env):
//   VITE_TRON_FEATURES=0     -> disable all hiding (stock upstream behaviour)
//   VITE_SKIP_ONBOARDING=1   -> never redirect to the onboarding wizard
//
// Ids: router paths start with "/", Settings sections are "settings:<id>",
// in-page panels/buttons are "feature:<id>".
export const TRON_HIDDEN_PAGES: readonly string[] = [
  // Onboarding wizard: its firmware-flash and GNSS-receiver steps shell out to
  // openocd/platformio and docker. Datum/dock are set in Settings instead.
  '/onboarding',
  // Settings sections
  'settings:updates',          // host updater (docker image pulls)
  'settings:remote_access',    // Tailscale sidecar created over the docker socket
  'settings:drive_motor',      // drive / PID auto-tuning (docker exec mowgli-ros2)
  // In-page features
  'feature:firmware_flash',    // dashboard "flash firmware" CTA
  'feature:gnss_configurator', // GNSS receiver plan/apply/factory-reset card (mowgli-gps container)
  'feature:host_updater',      // side-rail running-version / updates links
  'feature:rosbag',            // diagnostics rosbag recorder (docker exec mowgli-ros2)
];

const TRON_ENABLED = import.meta.env.VITE_TRON_FEATURES !== '0';

export const SKIP_ONBOARDING =
  import.meta.env.VITE_SKIP_ONBOARDING === '1' || (TRON_ENABLED && TRON_HIDDEN_PAGES.includes('/onboarding'));

export function isTronHidden(id: string | undefined): boolean {
  return TRON_ENABLED && id !== undefined && TRON_HIDDEN_PAGES.includes(id);
}
