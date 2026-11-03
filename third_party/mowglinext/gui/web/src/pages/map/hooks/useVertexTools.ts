import {useCallback, useMemo, useReducer} from "react";
import type {NotificationInstance} from "antd/es/notification/interface";
import {useTranslation} from "react-i18next";
import {
    BoundaryLineFeature,
    MowingFeatureBase,
    ObstacleFeature,
    featureFromJSON,
    serializeFeature,
    type MowingFeature,
} from "../../../types/map.ts";
import {
    initialVertexToolState,
    previewVertexTool,
    vertexToolReducer,
    type EditableGeometry,
    type VertexTool,
} from "./vertexToolState.ts";

/** Polygon (area / no-go / navigation) or boundary line: what the vertex tools edit. */
export const isVertexEditable = (f: MowingFeature | undefined): f is MowingFeature =>
    f instanceof MowingFeatureBase || f instanceof BoundaryLineFeature;

/**
 * A NEW instance of `feature` with `geometry`: DrawControl and the history
 * snapshot both key on instance identity, so an in-place edit would not sync.
 */
export function withGeometry(feature: MowingFeature, geometry: EditableGeometry): MowingFeature | null {
    const json = serializeFeature(feature);
    json.geometry = structuredClone(geometry);
    return featureFromJSON(json, feature instanceof ObstacleFeature ? feature.getMowingArea() : undefined);
}

interface UseVertexToolsOptions {
    features: Record<string, MowingFeature>;
    setFeatures: React.Dispatch<React.SetStateAction<Record<string, MowingFeature>>>;
    /** Enter edit mode first when applying from view mode (boundary line sheet). */
    ensureEditing: () => void;
    notification: NotificationInstance;
}

/**
 * "Merge nearby points" / "Simplify" dialog for one polygon or boundary
 * line. Apply replaces the feature in `features`, which the editor's history
 * records (undo / redo) and Save writes to map_server.
 */
export function useVertexTools({features, setFeatures, ensureEditing, notification}: UseVertexToolsOptions) {
    const {t} = useTranslation();
    const [state, dispatch] = useReducer(vertexToolReducer, undefined, initialVertexToolState);
    const target = state.featureId ? features[state.featureId] : undefined;
    const geometry = isVertexEditable(target) ? (target.geometry as EditableGeometry) : undefined;
    const preview = useMemo(() => (geometry ? previewVertexTool(state, geometry) : null), [state, geometry]);

    const open = useCallback((tool: VertexTool, featureId: string) => dispatch({type: "open", tool, featureId}), []);
    const close = useCallback(() => dispatch({type: "close"}), []);
    const setTolerance = useCallback((value: number | null) => dispatch({type: "tolerance", value}), []);

    const apply = useCallback(() => {
        if (!preview?.ok || !state.featureId || !state.tool) return;
        const id = state.featureId;
        const removed = preview.before - preview.after;
        if (removed > 0) {
            ensureEditing();
            setFeatures((curr) => {
                const f = curr[id];
                const next = f ? withGeometry(f, preview.geometry) : null;
                return next ? {...curr, [id]: next} : curr;
            });
        }
        notification.success({
            message: t(state.tool === "merge" ? "mapVertex.merged" : "mapVertex.simplified", {count: removed}),
        });
        dispatch({type: "close"});
    }, [preview, state.featureId, state.tool, ensureEditing, setFeatures, notification, t]);

    return {
        tool: state.tool,
        featureId: state.featureId,
        tolerance: state.tool ? state.tolerance[state.tool] : 0,
        preview,
        open,
        close,
        setTolerance,
        apply,
    };
}
