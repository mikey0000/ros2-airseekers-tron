import { beforeEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { MapToolbarMobile } from "./MapToolbarMobile.tsx";
import en from "../../../i18n/locales/en.json";

vi.mock("../../../theme/ThemeContext.tsx", () => ({
  useThemeMode: () => ({
    colors: {
      glassBackground: "#111",
      glassBorder: "1px solid #222",
      glassShadow: "none",
      bgElevated: "#222",
      danger: "#f00",
      text: "#fff",
    },
  }),
}));

vi.mock("../../../components/AsyncButton.tsx", () => ({
  default: ({ children, onAsyncClick, ...props }: any) => (
    <button onClick={onAsyncClick} {...props}>
      {children}
    </button>
  ),
}));

vi.mock("./ShapePickerDropdown.tsx", () => ({
  ShapePickerDropdown: ({ children }: any) => children,
}));

describe("MapToolbarMobile", () => {
  const defaultProps = {
    editMap: false,
    hasUnsavedChanges: false,
    manualMode: false,
    useSatellite: true,
    historyIndex: 0,
    editHistoryLength: 1,
    mowingAreas: [],
    stateName: "IDLE",
    highLevelState: 1,
    emergency: false,
    onEditMap: vi.fn(),
    onSaveMap: vi.fn().mockResolvedValue(undefined),
    onUndo: vi.fn(),
    onRedo: vi.fn(),
    onToggleSatellite: vi.fn(),
    onManualMode: vi.fn().mockResolvedValue(undefined),
    onStopManualMode: vi.fn().mockResolvedValue(undefined),
    onBackupMap: vi.fn(),
    onRestoreMap: vi.fn(),
    onDownloadGeoJSON: vi.fn(),
    onUploadGeoJSON: vi.fn(),
    onImportOpenMower: vi.fn(),
    onResetMowingProgress: vi.fn(),
    onMowArea: vi.fn().mockResolvedValue(undefined),
    onStart: vi.fn().mockResolvedValue(undefined),
    onHome: vi.fn().mockResolvedValue(undefined),
    onEmergencyOn: vi.fn().mockResolvedValue(undefined),
    onEmergencyOff: vi.fn().mockResolvedValue(undefined),
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("offers the reset action in the mobile data menu while idle", async () => {
    const user = userEvent.setup();
    render(<MapToolbarMobile {...defaultProps} />);

    await user.click(screen.getByLabelText(en.mapToolbarMobile.more));
    await user.click(screen.getByText(en.resetMowingProgress.action));

    expect(defaultProps.onResetMowingProgress).toHaveBeenCalledOnce();
  });

  it("opens area mow settings from the mobile toolbar", async () => {
    const user = userEvent.setup();
    const onAreaSettings = vi.fn();
    render(<MapToolbarMobile {...defaultProps}
      settingsAreas={[{ key: "a1", label: "Front lawn" }]}
      onAreaSettings={onAreaSettings} />);

    await user.click(screen.getByLabelText(en.mapToolbarMobile.areaSettings));
    await user.click(screen.getByText("Front lawn"));

    expect(onAreaSettings).toHaveBeenCalledWith("a1");
  });

  it("hides the area settings button without areas", () => {
    render(<MapToolbarMobile {...defaultProps} />);
    expect(screen.queryByLabelText(en.mapToolbarMobile.areaSettings)).toBeNull();
  });

  it("shows the Undock button only when docked", () => {
    const onUndock = vi.fn().mockResolvedValue(undefined);
    const { rerender } = render(<MapToolbarMobile {...defaultProps} stateName="IDLE_DOCKED" onUndock={onUndock} />);
    expect(screen.getByLabelText(en.mapToolbarMobile.undock)).toBeInTheDocument();
    rerender(<MapToolbarMobile {...defaultProps} stateName="CHARGING" onUndock={onUndock} />);
    expect(screen.getByLabelText(en.mapToolbarMobile.undock)).toBeInTheDocument();
    for (const s of ["IDLE", "MOWING", "UNDOCKING", "RECORDING"]) {
      rerender(<MapToolbarMobile {...defaultProps} stateName={s} onUndock={onUndock} />);
      expect(screen.queryByLabelText(en.mapToolbarMobile.undock)).toBeNull();
    }
  });
});
