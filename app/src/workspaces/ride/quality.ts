// 3D render quality. Without a GPU (a VM, a remote desktop, a headless CI runner) WebGL runs on
// a CPU rasterizer, and the full scene at device pixel ratio 2 can take most of the main thread.
// Low quality keeps the studio responsive there: 1x pixel ratio, no shadow maps, the contact
// shadow drawn once, and the scene redrawn at LOW_FPS instead of every display frame.

export type Quality = "high" | "low";

/**
 * Redraw rate in low quality. Under browser automation (CI screenshots on a GPU-less runner) the
 * scene draws at 5 fps: tests need the picture, not smooth motion, and every frame on a CPU
 * rasterizer delays input and DOM queries.
 */
export const LOW_FPS = typeof navigator !== "undefined" && navigator.webdriver ? 5 : 20;

const SOFTWARE_RENDERER = /swiftshader|llvmpipe|softpipe|software|basic render|lavapipe/i;

export function isSoftwareRenderer(renderer: string): boolean {
  return SOFTWARE_RENDERER.test(renderer);
}

/** The WebGL renderer string, when the browser exposes it. */
export function rendererName(gl: WebGLRenderingContext | WebGL2RenderingContext): string {
  const ext = gl.getExtension("WEBGL_debug_renderer_info");
  const name: unknown = ext
    ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL)
    : gl.getParameter(gl.RENDERER);
  return typeof name === "string" ? name : "";
}

/** `?quality=high|low` forces a quality (screenshots for docs, testing the low path on a GPU). */
export function qualityOverride(search: string): Quality | null {
  const q = new URLSearchParams(search).get("quality");
  return q === "high" || q === "low" ? q : null;
}

export function chooseQuality(renderer: string, override: Quality | null): Quality {
  return override ?? (isSoftwareRenderer(renderer) ? "low" : "high");
}
