import {act, render} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {useElementHeight} from "./useElementHeight.ts";

let lastHeight = -1;
function Probe({show}: {show: boolean}) {
    const [ref, h] = useElementHeight();
    lastHeight = h;
    return show ? <div ref={ref}>toolbar</div> : <span>loading</span>;
}

describe("useElementHeight", () => {
    it("measures an element that mounts after the first render", () => {
        const spy = vi.spyOn(HTMLElement.prototype, "getBoundingClientRect").mockReturnValue({height: 104} as DOMRect);
        const {rerender} = render(<Probe show={false}/>);
        expect(lastHeight).toBe(0);
        act(() => rerender(<Probe show/>));
        expect(lastHeight).toBe(104);
        act(() => rerender(<Probe show={false}/>));
        expect(lastHeight).toBe(0);
        spy.mockRestore();
    });
});
