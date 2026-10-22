import type {ModalFuncProps} from "antd";

/** The part of antd's App.useApp().modal this needs (easy to fake in tests). */
export type ConfirmModal = {confirm: (props: ModalFuncProps) => unknown};

export type ConfirmStartText = {title: string; content?: string; okText: string; cancelText: string};

/**
 * Fallback start path for when the StartMowSheet is not used (robot has no
 * area settings): never send a mission start from a single tap. Shows a
 * confirm dialog and runs `start` only on OK. Resolves when the dialog is
 * cancelled or the start succeeded; rejects with the start's error.
 */
export function confirmThenStart(modal: ConfirmModal, text: ConfirmStartText, start: () => Promise<unknown>): Promise<void> {
    return new Promise<void>((resolve, reject) => {
        modal.confirm({
            title: text.title,
            content: text.content,
            okText: text.okText,
            cancelText: text.cancelText,
            onOk: () => start().then(() => resolve(), (e: unknown) => reject(e instanceof Error ? e : new Error(String(e)))),
            onCancel: () => resolve(),
        });
    });
}
