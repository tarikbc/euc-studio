import { describe, expect, it } from "vitest";
import { isNavigationKey } from "./modality.ts";

const key = (k: string, mods: Partial<Record<"metaKey" | "ctrlKey" | "altKey", boolean>> = {}) => ({
  key: k,
  metaKey: false,
  ctrlKey: false,
  altKey: false,
  ...mods,
});

describe("isNavigationKey", () => {
  it("counts keys that move or act on focus", () => {
    for (const k of ["Tab", "ArrowLeft", "ArrowDown", "Home", "End", "Enter", " "]) {
      expect(isNavigationKey(key(k))).toBe(true);
    }
  });

  it("ignores shortcuts and plain typing", () => {
    expect(isNavigationKey(key("1", { metaKey: true }))).toBe(false);
    expect(isNavigationKey(key("k", { ctrlKey: true }))).toBe(false);
    expect(isNavigationKey(key("Tab", { altKey: true }))).toBe(false);
    expect(isNavigationKey(key("c"))).toBe(false);
    expect(isNavigationKey(key("Escape"))).toBe(false);
  });
});
