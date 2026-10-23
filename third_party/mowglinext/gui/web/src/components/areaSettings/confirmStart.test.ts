import {describe, expect, it, vi} from "vitest";
import type {ModalFuncProps} from "antd";
import {confirmThenStart} from "./confirmStart.ts";

const text = {title: "Start mowing?", okText: "Start", cancelText: "Cancel"};

function fakeModal() {
    const shown: ModalFuncProps[] = [];
    return {shown, modal: {confirm: (p: ModalFuncProps) => { shown.push(p); }}};
}

describe("confirmThenStart", () => {
    it("sends nothing until the user confirms", async () => {
        const {shown, modal} = fakeModal();
        const start = vi.fn().mockResolvedValue(undefined);
        const done = confirmThenStart(modal, text, start);
        expect(shown).toHaveLength(1);
        expect(shown[0].title).toBe("Start mowing?");
        expect(start).not.toHaveBeenCalled();
        await shown[0].onOk!();
        await done;
        expect(start).toHaveBeenCalledTimes(1);
    });

    it("cancel never starts", async () => {
        const {shown, modal} = fakeModal();
        const start = vi.fn().mockResolvedValue(undefined);
        const done = confirmThenStart(modal, text, start);
        shown[0].onCancel!();
        await done;
        expect(start).not.toHaveBeenCalled();
    });

    it("a failed start rejects so the caller can report it", async () => {
        const {shown, modal} = fakeModal();
        const done = confirmThenStart(modal, text, () => Promise.reject(new Error("boom")));
        await shown[0].onOk!();
        await expect(done).rejects.toThrow("boom");
    });
});
