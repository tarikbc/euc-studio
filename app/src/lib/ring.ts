// Fixed-capacity ring buffer over a Float64Array. Telemetry samples go in at up to 60 Hz (and,
// with the real engine, in binary batches); scopes copy the window out once per frame. Nothing
// here allocates after construction except `toArray()`.

export class RingBuffer {
  readonly capacity: number;
  private readonly data: Float64Array;
  private head = 0; // next write position
  private count = 0;
  private writes = 0;

  constructor(capacity: number) {
    if (!Number.isInteger(capacity) || capacity <= 0) {
      throw new RangeError(`RingBuffer capacity must be a positive integer, got ${capacity}`);
    }
    this.capacity = capacity;
    this.data = new Float64Array(capacity);
  }

  get length(): number {
    return this.count;
  }

  /** Changes on every write or clear; readers skip redrawing when it has not moved. */
  get version(): number {
    return this.writes;
  }

  push(value: number): void {
    this.data[this.head] = value;
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
    this.writes++;
  }

  /** Appends a batch (for example one columnar channel of an engine frame). */
  pushMany(values: ArrayLike<number>): void {
    // Only the newest `capacity` values can survive, so skip the rest up front.
    const start = Math.max(0, values.length - this.capacity);
    for (let i = start; i < values.length; i++) this.push(values[i] as number);
  }

  /** Value at `index`, where 0 is the oldest sample still held. */
  at(index: number): number | undefined {
    if (index < 0) index += this.count;
    if (index < 0 || index >= this.count) return undefined;
    return this.data[(this.head - this.count + index + this.capacity) % this.capacity];
  }

  /** Newest value, or `fallback` when empty. */
  last(fallback = 0): number {
    if (this.count === 0) return fallback;
    return this.data[(this.head - 1 + this.capacity) % this.capacity] as number;
  }

  /**
   * Copies the newest `max` samples (oldest first) into `out` starting at 0 and returns how many
   * were written. Two `set` calls at most, no allocation beyond the subarray views.
   */
  copyTo(out: Float64Array | Float32Array, max = this.count): number {
    const n = Math.min(max, this.count, out.length);
    if (n === 0) return 0;
    const start = (this.head - n + this.capacity) % this.capacity;
    const firstLen = Math.min(n, this.capacity - start);
    out.set(this.data.subarray(start, start + firstLen), 0);
    if (firstLen < n) out.set(this.data.subarray(0, n - firstLen), firstLen);
    return n;
  }

  toArray(): Float64Array {
    const out = new Float64Array(this.count);
    this.copyTo(out);
    return out;
  }

  clear(): void {
    this.head = 0;
    this.count = 0;
    this.writes++;
  }
}
