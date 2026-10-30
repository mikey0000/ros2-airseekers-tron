import {render, screen} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {SecureOriginCard, type SecureOriginStatus} from "./SecureOriginCard.tsx";

const enabledStatus: SecureOriginStatus = {
    enabled: true,
    https_urls: ["https://test-mower.local:4443", "https://192.0.2.10:4443"],
    hostname: "test-mower",
    ca_fingerprint_sha256: "AA:BB:CC",
    ca_not_after: "2036-10-27T12:00:00Z",
};

function mockStatus(status: SecureOriginStatus | null) {
    const fetchMock = vi.fn((input: string) => {
        expect(input).toBe("/api/secure-origin");
        return Promise.resolve(status
            ? new Response(JSON.stringify(status), {status: 200, headers: {"Content-Type": "application/json"}})
            : new Response("boom", {status: 500}));
    });
    vi.stubGlobal("fetch", fetchMock);
    return fetchMock;
}

function setSecureContext(value: boolean) {
    Object.defineProperty(window, "isSecureContext", {configurable: true, value});
}

describe("SecureOriginCard", () => {
    beforeEach(() => setSecureContext(false));
    afterEach(() => vi.unstubAllGlobals());

    it("lists the HTTPS URLs, the CA download and the fingerprint", async () => {
        const fetchMock = mockStatus(enabledStatus);
        render(<SecureOriginCard/>);

        const urls = await screen.findByTestId("secure-origin-urls");
        expect(Array.from(urls.querySelectorAll("a[href]")).map((a) => a.getAttribute("href")))
            .toEqual(enabledStatus.https_urls);
        expect(screen.getByTestId("secure-origin-download")).toHaveAttribute("href", "/api/secure-origin/ca.crt");
        expect(screen.getByText("AA:BB:CC")).toBeInTheDocument();
        expect(screen.getByTestId("secure-origin-verdict")).toHaveTextContent(/not on a secure origin/i);
        expect(fetchMock).toHaveBeenCalledTimes(1);
    });

    it("says so when the page already is on a secure origin", async () => {
        setSecureContext(true);
        mockStatus(enabledStatus);
        render(<SecureOriginCard/>);

        await screen.findByTestId("secure-origin-urls");
        expect(screen.getByTestId("secure-origin-verdict")).toHaveTextContent(/is on a secure origin/i);
    });

    it("explains when the mower could not set up HTTPS", async () => {
        mockStatus({enabled: false, https_urls: [], hostname: "test-mower"});
        render(<SecureOriginCard/>);

        expect(await screen.findByText(/HTTPS is not running/i)).toBeInTheDocument();
        expect(screen.queryByTestId("secure-origin-download")).toBeNull();
    });

    it("shows a load error instead of spinning forever", async () => {
        mockStatus(null);
        render(<SecureOriginCard/>);

        expect(await screen.findByText(/Could not read the HTTPS status/i)).toBeInTheDocument();
    });
});
