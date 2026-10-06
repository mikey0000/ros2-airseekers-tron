import {describe, expect, it} from "vitest";
import {DEFAULT_GEOMETRY, parseUrdf} from "./useRobotDescription.ts";

const urdf = (link: string) => `<robot>
  <joint name="left_wheel_joint" type="continuous">
    <origin xyz="0.1 0.2 0"/><parent link="base_link"/><child link="${link}"/>
  </joint>
  <link name="${link}"><visual><geometry><cylinder radius="0.0825" length="0.05"/></geometry></visual></link>
</robot>`;

describe("parseUrdf wheel link resolution", () => {
    it("follows the joint's child link when it is named left_wheel", () => {
        const g = parseUrdf(urdf("left_wheel"));
        expect(g.wheelRadius).toBe(0.0825);
        expect(g.wheelWidth).toBe(0.05);
        expect(g.wheelTrack).toBeCloseTo(0.4);
    });

    it("still works for left_wheel_link", () => {
        expect(parseUrdf(urdf("left_wheel_link")).wheelRadius).toBe(0.0825);
    });

    it("keeps defaults when the wheel link is absent", () => {
        expect(parseUrdf("<robot/>").wheelRadius).toBe(DEFAULT_GEOMETRY.wheelRadius);
    });
});
