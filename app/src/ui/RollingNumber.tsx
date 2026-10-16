import { AnimatePresence, type Variants, motion, useReducedMotion } from "motion/react";
import { useEffect, useEffectEvent, useRef, useState } from "react";
import { digitEnter, digitExit } from "../design/motion.ts";
import { digits } from "../design/tokens.ts";
import { cx } from "../lib/cx.ts";
import { useLowPower } from "../lib/power.ts";
import { formatNumber, throttleDelay, toSlots } from "./rolling.ts";

type Dir = 1 | -1;
/** `instant`: the digit changed again before its last roll finished, so it just updates. */
type Roll = { dir: Dir; instant: boolean };

const still = { y: 0, scale: 1, opacity: 1, filter: "blur(0px)" };

// Value going up: the new digit rises in from below and the old one leaves upward.
const rolling: Variants = {
  enter: ({ dir, instant }: Roll) =>
    instant
      ? still
      : {
          y: dir > 0 ? digits.enter.y : `-${digits.enter.y}`,
          scale: digits.enter.scale,
          opacity: 0,
          filter: `blur(${digits.enter.blur}px)`,
        },
  center: { ...still, transition: digitEnter },
  exit: ({ dir, instant }: Roll) =>
    instant
      ? { opacity: 0, transition: { duration: 0 } }
      : {
          y: dir > 0 ? `-${digits.exit.y}` : digits.exit.y,
          scale: digits.exit.scale,
          opacity: 0,
          filter: `blur(${digits.exit.blur}px)`,
          transition: digitExit,
        },
};

// Reduced motion: digits cross-fade in place.
const fading: Variants = {
  enter: ({ instant }: Roll) => ({ opacity: instant ? 1 : 0 }),
  center: { opacity: 1, transition: digitEnter },
  exit: ({ instant }: Roll) => ({ opacity: 0, transition: instant ? { duration: 0 } : digitExit }),
};

const ROLL_MS = digits.enter.duration * 1000;
const NONE: ReadonlySet<string> = new Set();

function DigitSlot({ char, roll, variants }: { char: string; roll: Roll; variants: Variants }) {
  return (
    <span className="relative inline-grid w-[1ch] justify-items-center">
      <AnimatePresence initial={false} custom={roll}>
        <motion.span
          key={char}
          custom={roll}
          variants={variants}
          initial="enter"
          animate="center"
          exit="exit"
          style={{ gridArea: "1 / 1" }}
        >
          {char}
        </motion.span>
      </AnimatePresence>
    </span>
  );
}

export interface RollingTextProps {
  text: string;
  /** +1 when the value went up, -1 when it went down. */
  direction?: Dir;
  /** Slot keys that should swap without rolling (they changed again mid-roll). */
  instant?: ReadonlySet<string>;
  className?: string;
}

/** Each digit has its own slot and rolls when it changes; other characters stay still. */
export function RollingText({ text, direction = 1, instant = NONE, className }: RollingTextProps) {
  const reduce = useReducedMotion();
  const lowPower = useLowPower();
  const variants = reduce || lowPower ? fading : rolling;
  return (
    <span className={cx("num relative inline-flex whitespace-pre", className)}>
      <span className="sr-only">{text}</span>
      <span aria-hidden="true" className="inline-flex">
        {toSlots(text).map((s) =>
          s.digit ? (
            <DigitSlot
              key={s.key}
              char={s.char}
              roll={{ dir: direction, instant: instant.has(s.key) }}
              variants={variants}
            />
          ) : (
            <span key={s.key} className="inline-block w-[1ch] text-center">
              {s.char}
            </span>
          ),
        )}
      </span>
    </span>
  );
}

export interface RollingNumberProps {
  value: number;
  decimals?: number;
  minIntegerDigits?: number;
  signed?: boolean;
  /** Custom text for the value (for example a clock); direction still follows the number. */
  format?: (value: number) => string;
  className?: string;
}

interface Shown {
  value: number;
  dir: Dir;
  instant: ReadonlySet<string>;
}

/**
 * A number whose digits roll. Updates are limited to 8 per second (the token), with a trailing
 * update so the final value always lands. A digit that changes again before its roll finishes
 * (a tenths digit at 8 Hz) swaps in place instead, so fast digits stay readable.
 */
export function RollingNumber({
  value,
  decimals = 0,
  minIntegerDigits = 1,
  signed = false,
  format,
  className,
}: RollingNumberProps) {
  const toText = (v: number) =>
    format ? format(v) : formatNumber(v, { decimals, minIntegerDigits, signed });
  const [shown, setShown] = useState<Shown>({ value, dir: 1, instant: NONE });
  const current = useRef(shown);
  const shownAt = useRef(0);
  const changedAt = useRef(new Map<string, number>());

  const apply = useEffectEvent(() => {
    const now = performance.now();
    shownAt.current = now;
    const prev = current.current;
    if (prev.value === value) return;
    const before = new Map(toSlots(toText(prev.value)).map((s) => [s.key, s.char]));
    const instant = new Set<string>();
    for (const s of toSlots(toText(value))) {
      if (before.get(s.key) === s.char) continue;
      if (now - (changedAt.current.get(s.key) ?? -Infinity) < ROLL_MS) instant.add(s.key);
      changedAt.current.set(s.key, now);
    }
    const next: Shown = { value, dir: value > prev.value ? 1 : -1, instant };
    current.current = next;
    setShown(next);
  });

  useEffect(() => {
    const wait = throttleDelay(shownAt.current, performance.now(), digits.maxUpdatesPerSecond);
    const id = window.setTimeout(apply, wait);
    return () => window.clearTimeout(id);
  }, [value]);

  return (
    <RollingText
      text={toText(shown.value)}
      direction={shown.dir}
      instant={shown.instant}
      className={className}
    />
  );
}
