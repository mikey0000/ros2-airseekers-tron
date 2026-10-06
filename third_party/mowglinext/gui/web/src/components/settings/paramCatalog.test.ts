import {describe, it, expect} from "vitest";
import {paramMeta, paramShortName, paramGroupLabelKey, PROFILE_CATALOG_OVERLAYS, TIER_RANK} from "./paramCatalog.ts";

describe("paramCatalog", () => {
  it("extracts the short name regardless of separator", () => {
    expect(paramShortName("fusion_graph_node.node_period_s")).toBe("node_period_s");
    expect(paramShortName("/map_server_node/tool_width")).toBe("tool_width");
    expect(paramShortName("tool_width")).toBe("tool_width");
  });

  it("resolves curated metadata by short name", () => {
    const meta = paramMeta("map_server_node.tool_width");
    expect(meta.tier).toBe("basic");
    expect(meta.group).toBe("Coverage");
    expect(meta.unit).toBe("m");
  });

  it("defaults unknown parameters to the expert tier and Other group", () => {
    const meta = paramMeta("some_node.totally_unknown_param");
    expect(meta.tier).toBe("expert");
    expect(meta.group).toBe("Other");
    expect(meta.label).toBe("totally_unknown_param");
  });

  it("orders tiers basic < middle < expert", () => {
    expect(TIER_RANK.basic).toBeLessThan(TIER_RANK.middle);
    expect(TIER_RANK.middle).toBeLessThan(TIER_RANK.expert);
  });
});

describe("paramCatalog profile overlays", () => {
  it("leaves stock profiles on the shared catalog", () => {
    expect(paramMeta("/behavior_tree_node.undock_distance_m").tier).toBe("expert");
    expect(paramMeta("/behavior_tree_node.undock_distance_m", "YardForce500").tier).toBe("expert");
    expect(paramMeta("map_server_node.tool_width", "YardForce500")).toEqual(paramMeta("map_server_node.tool_width"));
  });

  it("puts the Tron mission, nav and docking params in the basic tier", () => {
    const basic = [
      "/behavior_tree_node.undock_distance_m",
      "/behavior_tree_node.battery_low_percent",
      "/behavior_tree_node.rtk_timeout_s",
      "/behavior_tree_node.transit_gap_m",
      "/controller_server.FollowCoveragePath.desired_linear_vel",
      "/controller_server.FollowPath.desired_linear_vel",
      "/coverage_server.operation_width",
      "/mower_docking.approach_distance",
    ];
    for (const name of basic) {
      const meta = paramMeta(name, "AirseekersTron");
      expect(meta.tier, name).toBe("basic");
      expect(meta.label, name).toMatch(/^paramBindings\.catalog\./);
    }
  });

  it("tells the two desired_linear_vel apart by node-qualified key", () => {
    const mow = paramMeta("/controller_server.FollowCoveragePath.desired_linear_vel", "AirseekersTron");
    const transit = paramMeta("/controller_server.FollowPath.desired_linear_vel", "AirseekersTron");
    expect(mow.label).not.toBe(transit.label);
    // Some other node's desired_linear_vel stays uncurated.
    expect(paramMeta("/other_node.desired_linear_vel", "AirseekersTron").tier).toBe("expert");
  });

  it("uses the paramBindings namespace for overlay-only group headings", () => {
    expect(paramGroupLabelKey("Mission")).toBe("paramBindings.groups.Mission");
    expect(paramGroupLabelKey("Coverage")).toBe("paramGroups.Coverage");
  });

  it("every overlay label and group resolves in en.json", async () => {
    const en = (await import("../../i18n/locales/en.json")).default as Record<string, unknown>;
    const get = (path: string) => path.split(".").reduce<unknown>((o, k) => (o as Record<string, unknown> | undefined)?.[k], en);
    for (const overlay of Object.values(PROFILE_CATALOG_OVERLAYS)) {
      for (const meta of Object.values(overlay)) {
        expect(typeof get(meta.label), meta.label).toBe("string");
        expect(typeof get(meta.description), meta.description).toBe("string");
        expect(typeof get(paramGroupLabelKey(meta.group)), meta.group).toBe("string");
      }
    }
  });
});
