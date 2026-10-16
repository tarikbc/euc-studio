import type { ReactNode } from "react";
import { cx } from "../lib/cx.ts";

export interface PanelProps {
  /** A string becomes the panel's label; any other node (dock tabs) replaces it. */
  title: ReactNode;
  icon?: ReactNode;
  /** Right side of the header: controls, counts, pills. */
  actions?: ReactNode;
  children: ReactNode;
  className?: string;
  bodyClassName?: string;
}

/** A dock panel: a card with a 40px header (label + actions) and a scrollable body. */
export function Panel({ title, icon, actions, children, className, bodyClassName }: PanelProps) {
  return (
    <section
      className={cx(
        "flex h-full min-h-0 flex-col overflow-hidden rounded-12 border border-hairline bg-card",
        className,
      )}
    >
      <header className="flex h-(--dock-header) shrink-0 items-center gap-8 border-b border-hairline px-16">
        {icon && <span className="text-fg-3 [&_svg]:size-[14px]">{icon}</span>}
        {typeof title === "string" ? <h2 className="label truncate">{title}</h2> : title}
        <div className="ml-auto flex items-center gap-8">{actions}</div>
      </header>
      <div className={cx("min-h-0 flex-1 overflow-auto", bodyClassName)}>{children}</div>
    </section>
  );
}
