import { copyFileSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { type Page, expect, test } from "@playwright/test";

// Screenshots land in test-results/screens/<project>/ (gitignored). With DOCS_SCREENSHOTS=1
// the curated ones are also copied to docs/screenshots/ for the README.

const here = dirname(fileURLToPath(import.meta.url));
const DOCS = join(here, "..", "docs", "screenshots");
const CURATED = new Set(["ride", "gallery-components", "palette", "flash"]);

async function shot(page: Page, name: string, project: string) {
  const dir = join(here, "..", "test-results", "screens", project);
  mkdirSync(dir, { recursive: true });
  const file = join(dir, `${name}.png`);
  await page.screenshot({ path: file, animations: "disabled", caret: "hide" });
  if (
    process.env.DOCS_SCREENSHOTS &&
    CURATED.has(name) &&
    project === (process.env.DOCS_PROJECT ?? "chromium")
  ) {
    mkdirSync(DOCS, { recursive: true });
    copyFileSync(file, join(DOCS, `${name}.png`));
  }
  return file;
}

/**
 * Opens Ride and waits until it is ready. preroll fills the scopes with 14 s of the ride
 * scenario (cruising at 20 km/h). With `scene: false` the 3D view stays off (`?scene=off`):
 * tests about the shell do not wait for WebGL, which a slow runner draws in software. Headless
 * browsers render WebGL in software, so the app picks reduced 3D quality; README screenshots
 * force full quality instead.
 */
async function openRide(page: Page, { scene = true } = {}) {
  const quality = process.env.DOCS_SCREENSHOTS ? "&quality=high" : "";
  await page.goto(`/?preroll=14${scene ? "" : "&scene=off"}${quality}#/ride`);
  await waitForRide(page);
}

/** The shell is up and the 3D view has drawn its first frame (or is off). */
async function waitForRide(page: Page) {
  await expect(page.getByTestId("viewport")).toHaveAttribute("data-scene", /ready|off/, {
    timeout: 60_000,
  });
  await expect(page.getByText("Building the scene")).toHaveCount(0);
}

test.beforeEach(async ({ page, browserName }) => {
  if (process.env.PW_SLOW && browserName === "chromium") {
    const cdp = await page.context().newCDPSession(page);
    await cdp.send("Emulation.setCPUThrottlingRate", { rate: Number(process.env.PW_SLOW) || 4 });
  }
  // Start each test from defaults, but keep storage across reloads within the test.
  await page.addInitScript(() => {
    if (!sessionStorage.getItem("e2e-cleared")) {
      localStorage.clear();
      sessionStorage.setItem("e2e-cleared", "1");
    }
  });
});

test("ride workspace: viewport, telemetry, events", async ({ page }, info) => {
  await openRide(page);
  await expect(page.getByRole("radiogroup", { name: "Firmware" })).toBeVisible();
  await expect(page.getByRole("tab", { name: "Telemetry" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "Events" })).toBeVisible();
  await expect(page.getByText("First telemetry frame").first()).toBeVisible();
  await shot(page, "ride", info.project.name);
});

test("workspace switching with ⌘1-⌘4", async ({ page }, info) => {
  await openRide(page, { scene: false });
  for (const [key, name, heading] of [
    ["2", "tune", "See inside the firmware while it rides"],
    ["3", "compare", "Find where two firmwares part ways"],
    ["4", "flash", "Write firmware to a wheel, safely"],
  ] as const) {
    await page.keyboard.press(`ControlOrMeta+${key}`);
    await expect(page.getByRole("heading", { name: heading })).toBeVisible();
    await expect(page).toHaveURL(new RegExp(`#/${name}$`));
    await page.waitForTimeout(700);
    await shot(page, name, info.project.name);
  }
  await page.keyboard.press("ControlOrMeta+1");
  await expect(page.getByRole("button", { name: "Ride" })).toHaveAttribute("aria-current", "page");
});

test("docks toggle and persist", async ({ page }) => {
  await openRide(page, { scene: false });
  const right = page.getByTestId("dock-right");
  const width = async () => (await right.boundingBox())?.width ?? 0;
  await expect.poll(width).toBeGreaterThan(200);
  await page.keyboard.press("ControlOrMeta+b");
  await expect.poll(width).toBeLessThan(2);
  await page.reload();
  await waitForRide(page);
  await expect.poll(width).toBeLessThan(2);
  await page.keyboard.press("ControlOrMeta+b");
  await expect.poll(width).toBeGreaterThan(200);
});

test("command palette and shortcuts sheet", async ({ page }, info) => {
  await openRide(page, { scene: false });
  await page.keyboard.press("ControlOrMeta+k");
  const input = page.getByPlaceholder("Type a command or search");
  await expect(input).toBeFocused();
  await page.waitForTimeout(600);
  await shot(page, "palette", info.project.name);
  await input.fill("accent green");
  await page.keyboard.press("Enter");
  await expect(page.locator("html")).toHaveAttribute("data-accent", "green");
  await expect(page.getByText("Accent changed to Green").first()).toBeVisible();
  // The palette has finished closing (focus is back on the page) before the next shortcut.
  await expect(page.getByRole("dialog")).toHaveCount(0);

  await page.keyboard.press("?");
  await expect(page.getByRole("dialog", { name: "Keyboard shortcuts" })).toBeVisible();
  await page.waitForTimeout(600);
  await shot(page, "shortcuts", info.project.name);
  await page.keyboard.press("Escape");
  await expect(page.getByRole("dialog")).toHaveCount(0);
});

test("phone tab explains that it needs the live engine", async ({ page }, info) => {
  await openRide(page, { scene: false });
  await page.getByRole("tab", { name: "Phone" }).click();
  await expect(page.getByText("The phone needs the live engine")).toBeVisible();
  await shot(page, "phone-demo", info.project.name);
});

test("the event list fills again after a restart", async ({ page }, info) => {
  await openRide(page, { scene: false });
  const list = page.getByRole("heading", { name: "Events" }).locator("xpath=ancestor::section[1]");
  await expect(list.getByText("First telemetry frame")).toBeVisible();
  await page.keyboard.press("r");
  // The demo re-runs from 0 s: its boot events come back and their rows must show.
  await expect(list.getByText("First telemetry frame")).toBeVisible();
  await page.waitForTimeout(3000);
  await expect(list.getByText("First telemetry frame")).toBeVisible();
  await shot(page, "events-after-restart", info.project.name);
});

test("play, pause and the real-time factor", async ({ page }) => {
  await openRide(page, { scene: false });
  await page.getByRole("button", { name: "Pause simulation" }).click();
  await expect(page.getByText("Demo engine")).toBeVisible();
  await expect(page.getByText("paused", { exact: true })).toBeVisible();
  await page.keyboard.press("Space");
  await expect(page.getByText("running", { exact: true })).toBeVisible();
});

test("gallery", async ({ page }, info) => {
  await page.goto("/#/gallery");
  await expect(page.getByRole("heading", { name: "EUC Studio design system" })).toBeVisible();
  await page.waitForTimeout(800);
  await shot(page, "gallery", info.project.name);

  const scroller = page.getByTestId("gallery-scroll");
  for (const id of [
    "color",
    "contrast",
    "motion",
    "buttons",
    "controls",
    "feedback",
    "data",
    "overlays",
  ]) {
    await scroller.evaluate((el, sid) => {
      const target = el.querySelector(`#${sid}`) as HTMLElement | null;
      if (target) el.scrollTop = target.offsetTop - 24;
    }, id);
    await page.waitForTimeout(500);
    await shot(page, id === "buttons" ? "gallery-components" : `gallery-${id}`, info.project.name);
  }
});

test("reduced motion still renders the shell", async ({ page }, info) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  await openRide(page);
  await expect(page.getByRole("tab", { name: "Telemetry" })).toBeVisible();
  await shot(page, "ride-reduced-motion", info.project.name);
});
