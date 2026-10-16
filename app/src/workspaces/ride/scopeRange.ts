// The y range of a scope lane: the channel's own scale, widened just past the highest and lowest
// values of the session so a peak is never clipped. It never shrinks within a run.

/** Headroom past the session's extreme, as a share of the channel's own span. */
const HEADROOM = 0.1;

/** 1, 2 or 5 times a power of ten, at least `x`. */
export function niceStep(x: number): number {
  const p = 10 ** Math.floor(Math.log10(x));
  const f = x / p;
  return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p;
}

/**
 * The range for a channel scaled [min, max] whose session so far reached [seenLo, seenHi].
 * A side moves only when the data passes it, to the extreme plus 10 % of the channel's span,
 * rounded out to a round number so the axis labels stay clean.
 */
export function scopeRange(
  min: number,
  max: number,
  seenLo: number,
  seenHi: number,
): [number, number] {
  const span = max - min;
  const step = niceStep(span / 4);
  const hi = seenHi > max ? Math.ceil((seenHi + span * HEADROOM) / step) * step : max;
  const lo = seenLo < min ? Math.floor((seenLo - span * HEADROOM) / step) * step : min;
  return [lo, hi];
}
