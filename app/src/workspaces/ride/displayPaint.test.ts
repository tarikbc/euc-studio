import { describe, expect, it } from "vitest";
import type { DisplayPiece } from "../../engine/types.ts";
import { isLit } from "./displayPaint.ts";

describe("isLit", () => {
  // A big digit's segment a is wired to two grids (6 and 7); either one lights it.
  const segmentA: DisplayPiece = {
    kind: "polygon",
    points: [
      [0, 0],
      [1, 0],
      [1, 1],
    ],
    sources: [
      [6, 7],
      [7, 7],
    ],
  };

  it("lights a piece when any of its driver bits is set", () => {
    const grid = Array.from({ length: 16 }, () => 0);
    expect(isLit(segmentA, grid)).toBe(false);
    grid[7] = 0x80;
    expect(isLit(segmentA, grid)).toBe(true);
    grid[7] = 0x7f;
    expect(isLit(segmentA, grid)).toBe(false);
  });

  it("treats a short grid as dark", () => {
    expect(isLit(segmentA, [])).toBe(false);
  });
});
