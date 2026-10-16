// Runs the Tauri CLI (`pnpm tauri ...`). `tauri dev` gets the `unicorn` feature: the GPL-2.0
// CPU backend that runs the firmware until our own core lands (M3). It is for development only:
// set EUC_NO_UNICORN=1 to leave it out, and note that src-tauri refuses to compile it into any
// bundle (`tauri build`), so a release can never carry it.

import { spawn } from "node:child_process";

const args = process.argv.slice(2);
const hasFeatures = args.some(
    (a) => a === "--features" || a === "-f" || a.startsWith("--features="),
);
if (args[0] === "dev" && !process.env.EUC_NO_UNICORN && !hasFeatures) {
    args.push("--features", "unicorn");
}

const child = spawn("tauri", args, { stdio: "inherit", shell: process.platform === "win32" });
child.on("exit", (code, signal) => process.exit(code ?? (signal ? 1 : 0)));
