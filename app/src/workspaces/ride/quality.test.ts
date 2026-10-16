import { describe, expect, it } from "vitest";
import { chooseQuality, isSoftwareRenderer, qualityOverride } from "./quality.ts";

describe("render quality", () => {
  it("recognizes CPU rasterizers", () => {
    expect(
      isSoftwareRenderer(
        "ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (LLVM 10.0.0) (0x0000C0DE)), SwiftShader driver)",
      ),
    ).toBe(true);
    expect(isSoftwareRenderer("llvmpipe (LLVM 15.0.7, 256 bits)")).toBe(true);
    expect(isSoftwareRenderer("Microsoft Basic Render Driver")).toBe(true);
    expect(
      isSoftwareRenderer("ANGLE (Apple, ANGLE Metal Renderer: Apple M3 Max, Unspecified Version)"),
    ).toBe(false);
    expect(isSoftwareRenderer("Apple GPU")).toBe(false);
  });

  it("lets ?quality= override detection", () => {
    expect(qualityOverride("?preroll=14&quality=high")).toBe("high");
    expect(qualityOverride("?quality=ultra")).toBeNull();
    expect(chooseQuality("SwiftShader", "high")).toBe("high");
    expect(chooseQuality("Apple GPU", "low")).toBe("low");
    expect(chooseQuality("SwiftShader", null)).toBe("low");
    expect(chooseQuality("Apple GPU", null)).toBe("high");
  });
});
