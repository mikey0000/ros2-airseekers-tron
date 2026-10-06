import {Modal} from "antd";
import type {HookAPI} from "antd/es/modal/useModal";

export interface ConfirmSpec {
    title: string;
    content: string;
    okText: string;
    cancelText: string;
    danger?: boolean;
}

/** One confirm dialog shape for every map action that needs one (same as
 *  confirmUndock): resolves on cancel or once the action succeeded, rejects
 *  with the action's error. */
export const confirmAction = (
    modal: HookAPI | undefined, spec: ConfirmSpec, action: () => Promise<void>,
): Promise<void> => new Promise<void>((resolve, reject) => {
    (modal?.confirm ? modal : Modal).confirm({
        title: spec.title,
        content: spec.content,
        okText: spec.okText,
        okType: spec.danger ? "danger" : "primary",
        cancelText: spec.cancelText,
        onOk: () => action().then(resolve, (e: unknown) => { reject(e instanceof Error ? e : new Error(String(e))); }),
        onCancel: () => resolve(),
    });
});

/** Mowing phases in which "Dock" abandons the area being mowed (asks first). */
const DOCK_CONFIRM = new Set(["MOWING", "TRANSIT", "PLANNING", "UNDOCKING", "BOUNDARY_PAUSED", "AREA_UNREACHABLE"]);
export const dockNeedsConfirm = (stateName?: string): boolean => !!stateName && DOCK_CONFIRM.has(stateName);
