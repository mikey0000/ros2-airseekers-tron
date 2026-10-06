import {Modal} from "antd";
import type {TFunction} from "i18next";
import type {HookAPI} from "antd/es/modal/useModal";

/** Ask before undocking (the robot drives ~0.5 m forward off the dock).
 *  Resolves on cancel (no-op) or once the command was accepted; rejects on failure. */
export const confirmUndock = (
    modal: HookAPI | undefined, t: TFunction, onUndock: () => Promise<void>,
): Promise<void> => new Promise<void>((resolve, reject) => {
    (modal?.confirm ? modal : Modal).confirm({
        title: t("mapToolbar.undockConfirmTitle"),
        content: t("mapToolbar.undockConfirmContent"),
        okText: t("mapToolbar.undock"),
        cancelText: t("mapToolbar.undockCancel"),
        onOk: () => onUndock().then(resolve, (e: unknown) => { reject(e instanceof Error ? e : new Error(String(e))); }),
        onCancel: () => resolve(),
    });
});
