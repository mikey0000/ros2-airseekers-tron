import {
  Home, Map as MapIcon, Calendar, Compass, Settings, Terminal, Rocket, Activity,
  SlidersHorizontal, Camera,
} from "lucide-react";
import type {RobotProfile} from "../constants/robotProfiles.ts";
import {type GateId, isGateId, isGateVisible} from "../constants/profileGates.ts";

export interface NavItem {
  key: string;            // path
  labelKey: string;       // i18n key
  shortLabelKey?: string; // for the bottom-nav
  icon: typeof Home;
  showInBottom?: boolean;
}

/** Every destination the shell knows. Gated paths are filtered per robot. */
export const NAV_ITEMS: readonly NavItem[] = [
  {key: '/mowglinext',  labelKey: 'nav.home',        shortLabelKey: 'nav.home',      icon: Home,     showInBottom: true},
  {key: '/map',         labelKey: 'nav.map',                                          icon: MapIcon,  showInBottom: true},
  {key: '/schedule',    labelKey: 'nav.schedule',    shortLabelKey: 'nav.schedule',  icon: Calendar, showInBottom: true},
  {key: '/diagnostics', labelKey: 'nav.diagnostics', shortLabelKey: 'nav.diagShort', icon: Activity, showInBottom: true},
  {key: '/perception',  labelKey: 'profileGating.navPerception',                     icon: Camera,   showInBottom: false},
  {key: '/statistics',  labelKey: 'nav.stats',                                        icon: Compass,  showInBottom: false},
  {key: '/settings',    labelKey: 'nav.settings',                                     icon: Settings, showInBottom: false},
  {key: '/parameters',  labelKey: 'nav.parameters',                                   icon: SlidersHorizontal, showInBottom: false},
  {key: '/logs',        labelKey: 'nav.logs',                                         icon: Terminal, showInBottom: false},
  {key: '/onboarding',  labelKey: 'nav.onboarding',                                   icon: Rocket,   showInBottom: false},
];

/**
 * Nav entries to show. `isVisible` answers for gated paths only; ungated
 * paths are always shown. Pass `() => false` while the profile is loading.
 */
export function filterNavItems(isVisible: (path: GateId) => boolean): NavItem[] {
  return NAV_ITEMS.filter((n) => !isGateId(n.key) || isVisible(n.key));
}

/** Nav entries for a resolved profile (pure, for tests). */
export function navItemsFor(profile: RobotProfile): NavItem[] {
  return filterNavItems((path) => isGateVisible(profile, path));
}
