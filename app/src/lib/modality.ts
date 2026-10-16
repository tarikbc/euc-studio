// Which input the user used last, as `data-modality` on <html>: "keyboard" or "pointer".
//
// The focus ring follows it (design/global.css). :focus-visible alone is not enough in WebKit:
// Radix tabs, menus and dialogs move focus in code on a click, and after any key press (even a
// ⌘ shortcut) WebKit treats that focus as keyboard focus and draws the ring under the mouse.

/** Keys that move focus or act on the focused control. Shortcuts with ⌘/Ctrl/Alt do not count. */
const NAV_KEYS = new Set([
  "Tab",
  "ArrowUp",
  "ArrowDown",
  "ArrowLeft",
  "ArrowRight",
  "Home",
  "End",
  "PageUp",
  "PageDown",
  "Enter",
  " ",
]);

export function isNavigationKey(e: Pick<KeyboardEvent, "key" | "metaKey" | "ctrlKey" | "altKey">) {
  return !e.metaKey && !e.ctrlKey && !e.altKey && NAV_KEYS.has(e.key);
}

export function trackInputModality(root: HTMLElement = document.documentElement) {
  const set = (m: "keyboard" | "pointer") => {
    if (root.dataset.modality !== m) root.dataset.modality = m;
  };
  window.addEventListener("keydown", (e) => isNavigationKey(e) && set("keyboard"), true);
  window.addEventListener("pointerdown", () => set("pointer"), true);
}
