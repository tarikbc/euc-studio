// Reads a telemetry batch (crates/euc-telemetry/src/batch.rs) in place: the columns become typed
// array views over the message, no copies.

import {
  BATCH_FLAG_POSES,
  BATCH_HEADER_BYTES,
  BATCH_MAGIC,
  BATCH_VERSION,
  POSE_FLOATS,
} from "./telemetry.gen.ts";

export interface BatchView {
  seq: number;
  /** Samples the producer dropped before this batch because nobody read them. */
  dropped: number;
  /** Samples. */
  n: number;
  /** Channels per sample (the producer's schema length). */
  channels: number;
  /** Bodies per sample. */
  bodies: number;
  /** Sim time per sample, seconds. */
  t: Float64Array;
  /** Channel column `id` (length n). */
  column(id: number): Float32Array;
  /** All poses: n × bodies × 7 floats. */
  poses: Float32Array;
}

export class BatchError extends Error {}

export function readBatch(buffer: ArrayBuffer, byteOffset = 0): BatchView {
  if (buffer.byteLength - byteOffset < BATCH_HEADER_BYTES)
    throw new BatchError("batch is truncated");
  const view = new DataView(buffer, byteOffset, BATCH_HEADER_BYTES);
  if (view.getUint32(0, true) !== BATCH_MAGIC) throw new BatchError("not a telemetry batch");
  const version = view.getUint16(4, true);
  if (version !== BATCH_VERSION)
    throw new BatchError(`telemetry batch version ${version} is not supported`);
  const flags = view.getUint16(6, true);
  const n = view.getUint32(8, true);
  const channels = view.getUint32(12, true);
  const bodies = flags & BATCH_FLAG_POSES ? view.getUint32(16, true) : 0;
  const dropped = view.getUint32(20, true);
  const seq = view.getUint32(24, true);

  const tAt = byteOffset + BATCH_HEADER_BYTES;
  const columnsAt = tAt + 8 * n;
  const posesAt = columnsAt + 4 * n * channels;
  const end = posesAt + 4 * n * bodies * POSE_FLOATS;
  if (buffer.byteLength < end) throw new BatchError("batch is truncated");

  return {
    seq,
    dropped,
    n,
    channels,
    bodies,
    t: new Float64Array(buffer, tAt, n),
    column: (id) => new Float32Array(buffer, columnsAt + 4 * n * id, id < channels ? n : 0),
    poses: new Float32Array(buffer, posesAt, n * bodies * POSE_FLOATS),
  };
}
