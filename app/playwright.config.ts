import { defineConfig, devices } from "@playwright/test";

// Screenshots of the shell and the gallery against the web build (no Tauri). Chromium runs
// everywhere; WebKit (the engine Tauri uses on macOS and Linux) runs when PW_WEBKIT=1.
// PW_SLOW=1 approximates a GPU-less CI runner locally: software WebGL (--disable-gpu) plus 4x
// CPU throttling of the page (PW_SLOW=<n> for n x), applied per page in e2e/screens.spec.ts.
const CI = !!process.env.CI;
const SLOW = !!process.env.PW_SLOW;
const PORT = 4173;

export default defineConfig({
  testDir: "e2e",
  outputDir: "test-results/artifacts",
  fullyParallel: false,
  workers: 1,
  retries: CI ? 1 : 0,
  reporter: CI ? [["list"], ["html", { open: "never" }]] : "list",
  // A CI runner draws WebGL on the CPU and is slow at everything else too: tests wait on the
  // app's own ready signals, with room for a slow machine, rather than on fixed short sleeps.
  timeout: CI || SLOW ? 180_000 : 60_000,
  expect: { timeout: CI || SLOW ? 20_000 : 5_000 },
  use: {
    baseURL: `http://localhost:${PORT}`,
    viewport: { width: 1440, height: 900 },
    deviceScaleFactor: 2,
    colorScheme: "dark",
    trace: "retain-on-failure",
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        viewport: { width: 1440, height: 900 },
        deviceScaleFactor: 2,
        launchOptions: {
          args: [
            "--ignore-gpu-blocklist",
            "--enable-unsafe-swiftshader",
            ...(SLOW ? ["--disable-gpu"] : []),
          ],
        },
      },
    },
    ...(process.env.PW_WEBKIT
      ? [
          {
            name: "webkit",
            use: {
              ...devices["Desktop Safari"],
              viewport: { width: 1440, height: 900 },
              deviceScaleFactor: 2,
            },
          },
        ]
      : []),
  ],
  webServer: {
    command: CI ? "pnpm preview" : "pnpm build && pnpm preview",
    url: `http://localhost:${PORT}`,
    reuseExistingServer: !CI,
    timeout: 180_000,
  },
});
