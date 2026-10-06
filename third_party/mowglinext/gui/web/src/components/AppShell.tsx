import {type ReactNode, Suspense, useEffect, useMemo, useState} from "react";
import {useMatches, useNavigate, useOutlet} from "react-router-dom";
import {AnimatePresence, motion, LayoutGroup} from "framer-motion";
import {Spin} from "antd";
import {MoreHorizontal, X} from "lucide-react";

import {useTranslation} from "react-i18next";

import {MowerStatus} from "./MowerStatus.tsx";
import {NotificationBell} from "./NotificationBell.tsx";
import {LanguageSwitcher} from "./LanguageSwitcher.tsx";
import {LiveStatusStrip} from "./LiveStatusStrip.tsx";
import {ConnectionBadge} from "./ConnectionBadge.tsx";
import {IOSInstallBanner} from "./IOSInstallBanner.tsx";
import {useIOSInstallPrompt} from "../hooks/useIOSInstallPrompt.ts";
import {useAutoNotifications} from "../hooks/useNotificationCenter.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useHostUpdater} from "../hooks/useHostUpdater";
import {useIsMobile} from "../hooks/useIsMobile";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {BRAND_GRADIENT} from "../theme/colors.ts";
import {httpBase} from "../utils/apiHost.ts";
import {KEYFRAMES_CSS} from "./dashboard";
import "../concept/concept.css";
import {useGate, useProfileGates} from "../hooks/useProfileGates.ts";
import {filterNavItems, type NavItem} from "./navItems.ts";

/** Build-time escape hatch: never redirect to the onboarding wizard. */
const SKIP_ONBOARDING = import.meta.env.VITE_SKIP_ONBOARDING === '1';

/**
 * Premium tech-garden shell shared by the whole app.
 *
 * Desktop -> 88px fixed side-rail on the left + the page content in a
 *            comfortable max-width column with a sticky top status strip.
 * Mobile   -> bottom-nav with a sliding lime pill + slim top header.
 *
 * All surfaces inherit the /concept tokens (data-concept scope on body).
 */

// Nav entries live in ./navItems.ts; the ones whose path is a profile gate
// (constants/profileGates.ts) only show on robots that have the feature.

// Title falls back to the nav label key where they coincide; statistics has a
// fuller title than its short nav label.
const PAGE_META: Record<string, {titleKey: string; subtitleKey?: string}> = {
  '/mowglinext':  {titleKey: 'nav.home',                 subtitleKey: 'pageMeta.home.subtitle'},
  '/map':         {titleKey: 'nav.map',                  subtitleKey: 'pageMeta.map.subtitle'},
  '/schedule':    {titleKey: 'nav.schedule',             subtitleKey: 'pageMeta.schedule.subtitle'},
  '/diagnostics': {titleKey: 'nav.diagnostics',          subtitleKey: 'pageMeta.diagnostics.subtitle'},
  '/statistics':  {titleKey: 'pageMeta.statistics.title', subtitleKey: 'pageMeta.statistics.subtitle'},
  '/settings':    {titleKey: 'nav.settings',             subtitleKey: 'pageMeta.settings.subtitle'},
  '/parameters':  {titleKey: 'nav.parameters',           subtitleKey: 'pageMeta.parameters.subtitle'},
  '/logs':        {titleKey: 'nav.logs',                 subtitleKey: 'pageMeta.logs.subtitle'},
  '/perception':  {titleKey: 'profileGating.navPerception', subtitleKey: 'profileGating.perceptionSubtitle'},
  '/onboarding':  {titleKey: 'nav.onboarding'},
};

export function AppShell() {
  const {colors} = useThemeMode();
  const {t} = useTranslation();
  const navigate = useNavigate();
  const route = useMatches();
  const isMobile = useIsMobile();
  // Strict: a robot without a page never shows its entry, not even while
  // its profile is still loading.
  const {isVisibleStrict: gateVisible, loading: profileLoading} = useProfileGates();
  const nav = useMemo(() => filterNavItems(gateVisible), [gateVisible]);

  const currentPath = route.length > 1 ? route[1].pathname : '/mowglinext';
  const metaKeys = PAGE_META[currentPath];
  const meta = {
    title: metaKeys ? t(metaKeys.titleKey) : 'MowgliNext',
    subtitle: metaKeys?.subtitleKey ? t(metaKeys.subtitleKey) : undefined,
  };

  // Empty path -> dashboard. Without this `/` renders the shell with an
  // empty Outlet, which looks broken (the previous Root had this redirect
  // and we lost it in the AppShell rewrite).
  useEffect(() => {
    if (route.length === 1 && route[0].pathname === '/') {
      navigate({pathname: '/mowglinext'}, {replace: true});
    }
  }, [route, navigate]);

  // Onboarding gate (kept from the previous Root)
  const [configChecked, setConfigChecked] = useState(false);
  useEffect(() => {
    // Wait for the robot profile first (the wizard is profile-aware).
    if (configChecked || SKIP_ONBOARDING || profileLoading) return;
    (async () => {
      try {
        const res = await fetch(`${httpBase()}/api/settings/status`);
        const data = await res.json();
        if (!data.onboarding_completed && currentPath !== '/onboarding') {
          navigate({pathname: '/onboarding'});
        }
      } catch { /* ignore */ }
      setConfigChecked(true);
    })();
  }, [configChecked, currentPath, navigate, profileLoading]);

  // Auto-notifications hook (BT-state derived push notifications)
  const {highLevelStatus} = useHighLevelStatus();
  const emergency = useEmergency();
  const hwStatus = useStatus();
  useAutoNotifications({
    emergencyActive: highLevelStatus.emergency ?? emergency.active_emergency ?? false,
    emergencyLatched: emergency.latched_emergency ?? false,
    rainDetected: hwStatus.rain_detected ?? false,
    state: highLevelStatus.state_name,
  });

  // Bottom-nav items: the primary destinations live in the bar; the rest are
  // reachable through a "More" overflow sheet so nothing is unreachable on mobile.
  const bottomItems = useMemo(() => nav.filter(n => n.showInBottom), [nav]);
  const overflowItems = useMemo(() => nav.filter(n => !n.showInBottom), [nav]);
  const [moreOpen, setMoreOpen] = useState(false);

  // iOS "Add to Home Screen" hint — only ever shows on iOS Safari outside
  // standalone mode, and stays dismissed for the session (sessionStorage).
  const {showPrompt: showInstallBanner, dismiss: dismissInstallBanner} = useIOSInstallPrompt();

  if (isMobile) {
    return (
      <div data-concept style={{
        display: 'flex', flexDirection: 'column',
        // The frame never scrolls; only main does. Unlike hidden, clip also
        // prevents focused selects from shifting this frame horizontally.
        height: '100%', background: colors.bgBase, overflow: 'clip',
      }}>
        <style>{KEYFRAMES_CSS + PAGE_ENTER_CSS}</style>
        <AuroraBackdrop/>
        <LiveStatusStrip/>
        <ConnectionBadge/>

        <header style={{
          display: 'flex', flexWrap: 'wrap', alignItems: 'center', justifyContent: 'space-between',
          padding: '0 16px 6px',
          paddingTop: 'max(env(safe-area-inset-top, 0px), 6px)',
          minHeight: 56,
          background: 'rgba(2, 17, 13, 0.94)',
          borderBottom: `1px solid ${colors.borderSubtle}`,
          flexShrink: 0,
          position: 'relative', zIndex: 10,
        }}>
          <div>
            <div className="mn-display" style={{
              fontSize: 22, fontWeight: 400, color: colors.text,
              letterSpacing: '-0.01em', lineHeight: 1.1,
            }}>
              {meta.title}
            </div>
            {meta.subtitle && (
              <div style={{fontSize: 11, color: 'rgba(236, 255, 244, 0.42)', marginTop: 1}}>
                {meta.subtitle}
              </div>
            )}
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 6}}>
            <LanguageSwitcher/>
            <NotificationBell/>
          </div>
          <div style={{width:'100%', display:'flex', justifyContent:'flex-end', marginTop:4}}><MowerStatus/></div>
        </header>

        <main style={{
          flex: 1, overflow: 'auto', minHeight: 0,
          padding: '12px 14px calc(110px + env(safe-area-inset-bottom, 0px))',
          position: 'relative', zIndex: 1,
        }}>
          <AnimatedOutlet currentPath={currentPath}/>
        </main>

        {showInstallBanner && <IOSInstallBanner onDismiss={dismissInstallBanner}/>}

        <MobileMoreSheet
          open={moreOpen}
          items={overflowItems}
          activePath={currentPath}
          onClose={() => setMoreOpen(false)}
          onNavigate={(k) => { setMoreOpen(false); navigate({pathname: k}); }}
        />

        <MobileBottomNav
          items={bottomItems}
          activePath={currentPath}
          onNavigate={(k) => navigate({pathname: k})}
          onMore={() => setMoreOpen(true)}
          moreActive={overflowItems.some(n => n.key === currentPath) || moreOpen}
        />
      </div>
    );
  }

  // ─── Desktop ───
  return (
    <div data-concept style={{
      display: 'flex',
      height: '100%', minHeight: '100%', overflow: 'hidden',
      background: colors.bgBase,
      position: 'relative',
    }}>
      <style>{KEYFRAMES_CSS + PAGE_ENTER_CSS}</style>
      <AuroraBackdrop/>

      <DesktopSideRail
        items={nav}
        activePath={currentPath}
        onNavigate={(k) => navigate({pathname: k})}
      />

      <div style={{
        flex: 1, minWidth: 0, height: '100%',
        display: 'flex', flexDirection: 'column',
        marginLeft: 88,
        position: 'relative', zIndex: 1,
      }}>
        <LiveStatusStrip/>
        <ConnectionBadge/>
        <header style={{
          display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          padding: '18px 32px',
          background: 'rgba(2, 17, 13, 0.94)',
          borderBottom: `1px solid ${colors.borderSubtle}`,
          position: 'sticky', top: 0, zIndex: 30, overflow: 'visible',
        }}>
          <div>
            <div className="mn-display" style={{
              fontSize: 28, fontWeight: 400, color: colors.text,
              letterSpacing: '-0.015em', lineHeight: 1.05,
            }}>
              {meta.title}
            </div>
            {meta.subtitle && (
              <div style={{fontSize: 12, color: 'rgba(236, 255, 244, 0.42)', marginTop: 2}}>
                {meta.subtitle}
              </div>
            )}
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 12}}>
            <LanguageSwitcher/>
            <NotificationBell/>
            <MowerStatus/>
          </div>
        </header>
        <main style={{flex: 1, overflow: 'auto', minHeight: 0, padding: '24px 32px 48px'}}>
          <AnimatedOutlet currentPath={currentPath}/>
        </main>
      </div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Sub-shells
// ─────────────────────────────────────────────────────────────────────

function AuroraBackdrop() {
  return (
    <div aria-hidden style={{
      position: 'fixed', inset: -100, pointerEvents: 'none',
      background:
        "radial-gradient(circle at 18% 14%, rgba(69, 214, 232, 0.18) 0%, transparent 38%)," +
        "radial-gradient(circle at 80% 80%, rgba(124, 255, 178, 0.22) 0%, transparent 42%)," +
        "radial-gradient(circle at 50% 50%, rgba(107, 127, 255, 0.05) 0%, transparent 65%)",
      filter: 'blur(8px)', zIndex: 0,
    }}/>
  );
}

// Page entry animation. Deliberately plain CSS rather than framer-motion's
// AnimatePresence: with mode="wait" the next page was only mounted once the
// previous page's exit animation reported completion, so whenever frame
// callbacks stalled (background or occluded tab, throttled kiosk/WebView,
// automation) the header followed the route while the body stayed frozen on
// the page that was exiting — and a freshly loaded page stayed at its initial
// opacity:0/translateY state. CSS animations run on the document timeline, so
// a late frame simply paints the finished state, and the route swap itself no
// longer waits on any animation.
const PAGE_ENTER_CSS = `
@keyframes mn-page-enter { from { opacity: 0; transform: translateY(10px); } }
.mn-page-enter { animation: mn-page-enter 0.28s cubic-bezier(0.2, 0.7, 0.2, 1); }
@media (prefers-reduced-motion: reduce) { .mn-page-enter { animation: none; } }
`;

function AnimatedOutlet({currentPath}: {currentPath: string}) {
  const outlet = useOutlet();
  // Scrolling pages must grow with their content so main's bottom padding
  // follows the final control instead of sitting behind overflowing children.
  // Map and logs own their viewport layout and still need a definite height.
  const fillViewport = currentPath === '/map' || currentPath === '/logs';
  return (
    // Keyed by path: each page mounts fresh (as before) and replays the entry
    // animation. Exactly one wrapper exists at a time, so no outgoing page can
    // sit above the new one and push it down.
    <div
      key={currentPath}
      className="mn-page-enter"
      data-testid="page-outlet"
      style={{minHeight: '100%', height: fillViewport ? '100%' : undefined}}
    >
      {/* Pages are React.lazy chunks. Suspend here, inside the shell, rather
          than at the root boundary above RouterProvider, which would swap the
          whole app (rail and header included) for a full-screen spinner. */}
      <Suspense fallback={<div style={{display: 'flex', justifyContent: 'center', padding: 48}}><Spin size="large"/></div>}>
        {outlet}
      </Suspense>
    </div>
  );
}

// ─── Side-rail ───
interface RailProps {
  items: NavItem[];
  activePath: string;
  onNavigate: (k: string) => void;
  onMore?: () => void;
  moreActive?: boolean;
}

function DesktopSideRail({items, activePath, onNavigate}: RailProps) {
  const {t} = useTranslation();
  const navigate = useNavigate();
  const hostUpdater = useGate('feature:host_updater');
  return (
    <aside style={{
      position: 'fixed', top: 0, bottom: 0, left: 0, width: 88,
      display: 'flex', flexDirection: 'column',
      paddingTop: 24, paddingBottom: 24,
      background: 'linear-gradient(180deg, rgba(2, 17, 13, 0.92), rgba(2, 17, 13, 0.84))',
      borderRight: '1px solid rgba(236, 255, 244, 0.07)',
      zIndex: 40,
    }}>
      <div style={{
        display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6,
        marginBottom: 24,
      }}>
        <div style={{
          width: 44, height: 44, borderRadius: 14,
          background: BRAND_GRADIENT,
          color: '#02110D',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          fontFamily: 'Satoshi', fontWeight: 900, fontSize: 22, lineHeight: 1,
          boxShadow: '0 10px 24px -8px rgba(124, 255, 178, 0.5)',
        }}>
          m
        </div>
        <div style={{
          fontSize: 9, color: 'rgba(236, 255, 244, 0.42)',
          letterSpacing: '0.18em', textTransform: 'uppercase', fontWeight: 700,
        }}>
          Mowgli
        </div>
      </div>

      <LayoutGroup>
        <nav style={{display: 'flex', flexDirection: 'column', gap: 4, padding: '0 12px', flex: 1, overflowY: 'auto'}}>
          {items.map(({key, labelKey, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button
                key={key}
                onClick={() => onNavigate(key)}
                aria-label={t(labelKey)}
                aria-current={isActive ? 'page' : undefined}
                style={{
                  position: 'relative',
                  display: 'flex', flexDirection: 'column',
                  alignItems: 'center', justifyContent: 'center', gap: 4,
                  padding: '12px 4px 10px',
                  borderRadius: 14,
                  background: 'transparent',
                  border: 'none', cursor: 'pointer',
                  color: isActive ? '#02110D' : 'rgba(236, 255, 244, 0.62)',
                  fontSize: 9, fontWeight: 700,
                  letterSpacing: '0.06em', textTransform: 'uppercase',
                  zIndex: 1,
                  transition: 'color 0.15s',
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="app-rail-pill"
                    style={{
                      position: 'absolute', inset: 0,
                      background: BRAND_GRADIENT,
                      borderRadius: 14,
                      boxShadow: '0 12px 26px -6px rgba(124, 255, 178, 0.5), inset 0 1px 0 rgba(255, 255, 255, 0.32)',
                      zIndex: -1,
                    }}
                    transition={{type: 'spring', stiffness: 380, damping: 32}}
                  />
                )}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{t(labelKey)}</span>
              </button>
            );
          })}
        </nav>
      </LayoutGroup>
      {hostUpdater && <RunningVersionSummary onClick={() => void navigate('/settings?section=updates')}/>}
    </aside>
  );
}

// The footer describes the verified running release, never the update target.
function RunningVersionSummary({onClick, mobile = false}: {onClick: () => void; mobile?: boolean}) {
  const {t} = useTranslation();
  const {data, error} = useHostUpdater();
  const active = data?.state.active;
  const matched = !error && data?.runtime?.identity === 'matched' && active;
  const version = matched ? (active.source.track === 'stable' ? active.release_tag || active.id : active.revision.slice(0, 8))
    : t(!error && ['mixed', 'custom', 'drifted'].includes(data?.runtime?.identity ?? '') ? 'hostUpdater.summaryCustom' : 'hostUpdater.summaryUnknown');
  const track = matched ? (active.source.track === 'custom' ? active.source.branch : t(`hostUpdater.tracks.${active.source.track}`)) : t('hostUpdater.installed');
  const description = t('hostUpdater.runningSummary', {version, track});
  return <button data-testid="running-version-summary" onClick={onClick} aria-label={description} title={description} style={{
    display:'flex', flexDirection:mobile ? 'row' : 'column', alignItems:'center', justifyContent:'center', gap:5,
    margin:mobile ? 0 : '12px 8px 0', padding:'12px 2px', minHeight:48, gridColumn:'1 / -1',
    background:'transparent', border:'none', borderTop:'1px solid rgba(236,255,244,0.1)',
    color:'#7CFFB2', cursor:'pointer', overflowWrap:'anywhere',
  }}><span style={{fontSize:12, fontWeight:700}}>{version}</span><span style={{fontSize:10, color:'rgba(236,255,244,0.62)'}}>{track}</span></button>;
}

// ─── Mobile bottom nav ───
const bottomNavBtnStyle = (isActive: boolean): React.CSSProperties => ({
  position: 'relative',
  display: 'flex', flexDirection: 'column',
  alignItems: 'center', justifyContent: 'center', gap: 2,
  padding: '10px 4px 8px',
  borderRadius: 999,
  background: 'transparent', border: 'none', cursor: 'pointer',
  color: isActive ? '#02110D' : 'rgba(236, 255, 244, 0.66)',
  fontSize: 10, fontWeight: 600,
  letterSpacing: '0.02em',
  zIndex: 1,
  transition: 'color 0.15s',
});

const bottomNavPill = (
  <motion.span
    layoutId="app-bottom-pill"
    style={{
      position: 'absolute', inset: 0,
      background: BRAND_GRADIENT,
      borderRadius: 999,
      boxShadow: '0 6px 20px -6px rgba(124, 255, 178, 0.55), inset 0 1px 0 rgba(255, 255, 255, 0.3)',
      zIndex: -1,
    }}
    transition={{type: 'spring', stiffness: 380, damping: 32}}
  />
);

function MobileBottomNav({items, activePath, onNavigate, onMore, moreActive}: RailProps) {
  const {t} = useTranslation();
  const {displayMode} = useThemeMode();
  const columns = items.length + (onMore ? 1 : 0);
  return (
    <nav style={{
      position: 'fixed', left: 0, right: 0, bottom: 0,
      paddingBottom: 'calc(env(safe-area-inset-bottom, 0px) + 10px)',
      paddingTop: 10,
      paddingLeft: 14, paddingRight: 14,
      background: 'linear-gradient(180deg, rgba(2, 17, 13, 0) 0%, rgba(2, 17, 13, 0.85) 30%, rgba(2, 17, 13, 0.97) 100%)',
      backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined,
      WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined,
      zIndex: 50,
    }}>
      <LayoutGroup>
        <div style={{
          display: 'grid',
          gridTemplateColumns: `repeat(${columns}, 1fr)`,
          gap: 2, padding: 6,
          background: 'rgba(255, 255, 255, 0.04)',
          border: '1px solid rgba(236, 255, 244, 0.08)',
          borderRadius: 999,
          backdropFilter: displayMode === 'visual' ? 'blur(28px)' : undefined,
          WebkitBackdropFilter: displayMode === 'visual' ? 'blur(28px)' : undefined,
        }}>
          {items.map(({key, labelKey, shortLabelKey, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button key={key} onClick={() => onNavigate(key)} aria-label={t(labelKey)} style={bottomNavBtnStyle(isActive)}>
                {isActive && bottomNavPill}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{t(shortLabelKey ?? labelKey)}</span>
              </button>
            );
          })}
          {onMore && (
            <button onClick={onMore} aria-label={t('nav.more')} style={bottomNavBtnStyle(!!moreActive)}>
              {moreActive && bottomNavPill}
              <MoreHorizontal size={18} strokeWidth={moreActive ? 2.4 : 2}/>
              <span>{t('nav.more')}</span>
            </button>
          )}
        </div>
      </LayoutGroup>
    </nav>
  );
}

// ─── Mobile "More" overflow sheet ───
interface MoreSheetProps {
  open: boolean;
  items: NavItem[];
  activePath: string;
  onClose: () => void;
  onNavigate: (k: string) => void;
}

function MobileMoreSheet({open, items, activePath, onClose, onNavigate}: MoreSheetProps) {
  const {t} = useTranslation();
  const navigate = useNavigate();
  const hostUpdater = useGate('feature:host_updater');
  const {displayMode} = useThemeMode();

  // Escape closes the sheet (keyboard parity with the backdrop tap / X button).
  useEffect(() => {
    if (!open) return;
    const handler = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [open, onClose]);

  return (
    <AnimatePresence>
      {open && (
        <>
          <motion.div
            initial={{opacity: 0}} animate={{opacity: 1}} exit={{opacity: 0}}
            onClick={onClose}
            style={{position: 'fixed', inset: 0, background: 'rgba(2, 17, 13, 0.6)', zIndex: 60}}
          />
          <motion.div
            initial={{y: '100%'}} animate={{y: 0}} exit={{y: '100%'}}
            transition={{type: 'spring', stiffness: 420, damping: 38}}
            style={{
              position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 61,
              padding: '14px 14px calc(env(safe-area-inset-bottom, 0px) + 18px)',
              background: 'rgba(6, 24, 18, 0.97)',
              backdropFilter: displayMode === 'visual' ? 'blur(24px) saturate(140%)' : undefined,
              WebkitBackdropFilter: displayMode === 'visual' ? 'blur(24px) saturate(140%)' : undefined,
              borderTop: '1px solid rgba(236, 255, 244, 0.1)',
              borderRadius: '20px 20px 0 0',
            }}
          >
            <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12}}>
              <span style={{fontSize: 13, fontWeight: 600, color: 'rgba(236, 255, 244, 0.66)'}}>{t('nav.more')}</span>
              <button onClick={onClose} aria-label={t('nav.close')} style={{
                background: 'transparent', border: 'none', cursor: 'pointer', color: 'rgba(236, 255, 244, 0.66)',
                display: 'flex', padding: 4,
              }}>
                <X size={20}/>
              </button>
            </div>
            <div style={{display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: 10}}>
              {items.map(({key, labelKey, icon: Icon}) => {
                const isActive = key === activePath;
                return (
                  <button key={key} onClick={() => onNavigate(key)} style={{
                    display: 'flex', alignItems: 'center', gap: 12,
                    padding: '14px 16px', borderRadius: 14, cursor: 'pointer',
                    background: isActive ? 'rgba(124, 255, 178, 0.14)' : 'rgba(255, 255, 255, 0.04)',
                    border: `1px solid ${isActive ? 'rgba(124, 255, 178, 0.4)' : 'rgba(236, 255, 244, 0.08)'}`,
                    color: isActive ? '#7CFFB2' : 'rgba(236, 255, 244, 0.82)',
                    fontSize: 14, fontWeight: 600,
                  }}>
                    <Icon size={20}/>
                    <span>{t(labelKey)}</span>
                  </button>
                );
              })}
              {hostUpdater && <RunningVersionSummary mobile onClick={() => { onClose(); void navigate('/settings?section=updates'); }}/>}

            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default AppShell;

// Type alias for callers that previously imported Root.
export {AppShell as Root};
export type {ReactNode};
