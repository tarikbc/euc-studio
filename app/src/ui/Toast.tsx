import { motion } from "motion/react";
import { Toast as T } from "radix-ui";
import type { ReactNode } from "react";
import { bouncy } from "../design/motion.ts";
import { cx } from "../lib/cx.ts";
import { CircleCheck, Info, OctagonAlert, TriangleAlert, X } from "./icons.tsx";
import { type ToastItem, type ToastTone, useToasts } from "./toast-store.ts";

const toneIcon: Record<ToastTone, { icon: ReactNode; className: string }> = {
  success: { icon: <CircleCheck />, className: "text-ok" },
  info: { icon: <Info />, className: "text-info" },
  caution: { icon: <TriangleAlert />, className: "text-caution" },
  danger: { icon: <OctagonAlert />, className: "text-danger-text" },
};

/** The visual card, shared by live toasts and the gallery. */
export function ToastCard({
  item,
  onClose,
  className,
}: {
  item: Omit<ToastItem, "id">;
  onClose?: () => void;
  className?: string;
}) {
  const ic = toneIcon[item.tone];
  return (
    <div
      className={cx(
        "flex w-[340px] items-start gap-8 rounded-12 border border-hairline-strong bg-raised p-16 shadow-overlay",
        className,
      )}
    >
      <motion.span
        className={cx("mt-[1px] shrink-0 [&_svg]:size-[16px]", ic.className)}
        initial={item.tone === "success" ? { scale: 0.4, opacity: 0 } : false}
        animate={{ scale: 1, opacity: 1 }}
        transition={bouncy}
      >
        {ic.icon}
      </motion.span>
      <div className="min-w-0 flex-1">
        <p className="text-13 font-semibold text-fg">{item.title}</p>
        {item.description && <p className="mt-2 text-12 text-fg-2-safe">{item.description}</p>}
        {item.action && (
          <button
            type="button"
            onClick={item.action.run}
            className="pressable mt-8 rounded-4 text-12 font-semibold text-accent-text live:hover:underline"
          >
            {item.action.label}
          </button>
        )}
      </div>
      {onClose && (
        <button
          type="button"
          aria-label="Dismiss"
          onClick={onClose}
          className="pressable -mt-4 -mr-8 grid size-(--control-sm) place-items-center rounded-8 text-fg-3 live:hover:bg-track live:hover:text-fg [&_svg]:size-[14px]"
        >
          <X />
        </button>
      )}
    </div>
  );
}

// Under browser automation a toast stays until dismissed, so a slow test runner cannot miss a
// confirmation that already came and went. People get the usual 4.5 s.
const DURATION = typeof navigator !== "undefined" && navigator.webdriver ? Infinity : 4500;

export function Toaster() {
  const { items, dismiss } = useToasts();
  return (
    <T.Provider swipeDirection="right" duration={DURATION}>
      {items.map((item) => (
        <T.Root
          key={item.id}
          onOpenChange={(open) => !open && dismiss(item.id)}
          className="toast"
          type={item.tone === "danger" ? "foreground" : "background"}
        >
          <T.Title className="sr-only">{item.title}</T.Title>
          {item.description && (
            <T.Description className="sr-only">{item.description}</T.Description>
          )}
          <ToastCard item={item} onClose={() => dismiss(item.id)} />
        </T.Root>
      ))}
      <T.Viewport className="fixed right-16 bottom-[44px] z-[60] flex max-w-[100vw] flex-col gap-8 outline-none" />
    </T.Provider>
  );
}
