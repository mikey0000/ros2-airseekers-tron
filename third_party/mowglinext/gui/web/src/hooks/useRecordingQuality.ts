import {useCallback, useEffect, useState} from "react";
import {useApi} from "./useApi.ts";
import {parseRecordingQuality, type RecordingQuality} from "../utils/recordingQuality.ts";

/**
 * Recording sidecars (GET /mowglinext/recording-quality). Fetched on mount and
 * whenever `reloadKey` changes (the caller passes something that changes when a
 * recording is saved, e.g. the map). Empty when the backend has none or fails.
 */
export function useRecordingQuality(enabled: boolean, reloadKey: unknown): RecordingQuality[] {
    const guiApi = useApi();
    const [recordings, setRecordings] = useState<RecordingQuality[]>([]);

    const load = useCallback(async (): Promise<RecordingQuality[]> => {
        try {
            const res = await guiApi.request<unknown>({
                path: "/mowglinext/recording-quality",
                method: "GET",
                format: "json",
            });
            return parseRecordingQuality(res.data);
        } catch {
            return [];
        }
    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    useEffect(() => {
        if (!enabled) {
            setRecordings([]);
            return;
        }
        let live = true;
        void load().then((r) => { if (live) setRecordings(r); });
        return () => { live = false; };
    }, [enabled, reloadKey, load]);

    return recordings;
}
