import {useTopic} from "./useTopic.ts";
import {parseCutterHeightMsg} from "../utils/bladeHeight.ts";

/**
 * Commanded deck height in mm (latched std_msgs/Int16 /cutter/height_mm from
 * mcu_node). There is no deck-position telemetry: this is the last height the
 * driver sent. null until the first message.
 */
export const useCutterHeight = (enabled = true): number | null =>
    useTopic<number | null>("cutterHeight", null, {enabled, select: parseCutterHeightMsg}).data;
