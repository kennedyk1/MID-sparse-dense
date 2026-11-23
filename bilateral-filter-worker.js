"use strict";

function parseCalibration(text) {
  const lines = text.split(/\r?\n/).map(line => line.replace(/#.*$/, "").trim()).filter(Boolean);
  const field = key => lines.find(line => line.startsWith(`${key}:`))?.slice(key.length + 1).trim();
  function matrix(key, size) {
    const start = lines.indexOf(`${key}:`) + 1;
    if (!start) throw new Error(`Missing calibration matrix ${key}.`);
    const rows = lines.slice(start, start + size).map(line => JSON.parse(line.replace(/^-\s+/, "")));
    if (rows.length !== size || !rows.every(row => row.length === size && row.every(Number.isFinite))) {
      throw new Error(`Invalid calibration matrix ${key}.`);
    }
    return rows;
  }

  const [width, height] = JSON.parse(field("image_size_px"));
  const K = matrix("K", 3);
  const D = JSON.parse(field("D"));
  const transform = matrix("T_color_from_lidar", 4);
  if (![width, height].every(value => Number.isInteger(value) && value > 0) ||
      field("camera_model") !== "pinhole" || field("distortion_model") !== "opencv_radial_tangential" ||
      field("translation_unit") !== "m" || D.length !== 5 || !D.every(Number.isFinite) ||
      K[0][0] <= 0 || K[1][1] <= 0 || transform[3].some((value, i) => value !== Number(i === 3))) {
    throw new Error("Unsupported camera or LiDAR calibration.");
  }
  return { width, height, K, D, transform };
}

function readPly(buffer) {
  const header = new TextDecoder().decode(new Uint8Array(buffer, 0, Math.min(buffer.byteLength, 65536)));
  const end = /(?:^|\n)end_header\r?\n/.exec(header);
  if (!end) throw new Error("Missing PLY header terminator.");
  const lines = header.slice(0, end.index).split(/\r?\n/);
  if (lines[0] !== "ply" || lines[1] !== "format binary_little_endian 1.0") {
    throw new Error("Expected a binary little-endian PLY.");
  }
  const properties = [];
  let count;
  for (const line of lines.slice(2)) {
    const [declaration, type, name] = line.trim().split(/\s+/);
    if (declaration === "element") {
      if (type !== "vertex" || count !== undefined) throw new Error("Expected a vertex-only PLY.");
      count = Number(name);
    } else if (declaration === "property") {
      if (count === undefined || !["float", "float32"].includes(type)) {
        throw new Error("Expected float32 PLY vertex fields.");
      }
      properties.push(name);
    }
  }
  const offset = end.index + end[0].length;
  const stride = properties.length * 4;
  const xyzOffsets = ["x", "y", "z"].map(name => properties.indexOf(name) * 4);
  if (!Number.isInteger(count) || count < 0 || xyzOffsets.some(value => value < 0) ||
      offset + count * stride !== buffer.byteLength) {
    throw new Error("Invalid PLY vertex data.");
  }
  return { count, stride, xyzOffsets, data: new DataView(buffer, offset) };
}

function distort(x, y, [k1, k2, p1, p2, k3]) {
  const xx = x * x;
  const yy = y * y;
  const xy = x * y;
  const r2 = xx + yy;
  return [1 + r2 * (k1 + r2 * (k2 + r2 * k3)), 2 * p1 * xy + p2 * (r2 + 2 * xx), p1 * (r2 + 2 * yy) + 2 * p2 * xy];
}

function agreesWithInverseLens(dx, dy, x, y, D) {
  let rx = dx;
  let ry = dy;
  // The same inverse-distortion check used by the Python projection rejects folded rays.
  for (let i = 0; i < 50; i++) {
    const [radial, tx, ty] = distort(rx, ry, D);
    if (!(radial > 0)) return false;
    const nextX = (dx - tx) / radial;
    const nextY = (dy - ty) / radial;
    const converged = Math.abs(nextX - rx) < 1e-13 && Math.abs(nextY - ry) < 1e-13;
    rx = nextX;
    ry = nextY;
    if (converged) break;
  }
  return Math.abs(rx - x) <= 1e-5 && Math.abs(ry - y) <= 1e-5;
}

function prepareFrame(ply, calibration) {
  const { count, stride, xyzOffsets: [ix, iy, iz], data } = readPly(ply);
  const { width, height, K, D, transform: [a, b, c] } = calibration;
  const [fx, , cx] = K[0];
  const [, fy, cy] = K[1];
  const hasDistortion = D.some(value => value !== 0);
  const projected = [];
  for (let offset = 0; offset < count * stride; offset += stride) {
    const x = data.getFloat32(offset + ix, true);
    const y = data.getFloat32(offset + iy, true);
    const z = data.getFloat32(offset + iz, true);
    const range = Math.hypot(x, y, z);
    if (!Number.isFinite(range) || range <= 0) continue;
    const depth = c[0] * x + c[1] * y + c[2] * z + c[3];
    if (depth <= 0) continue;
    const nx = (a[0] * x + a[1] * y + a[2] * z + a[3]) / depth;
    const ny = (b[0] * x + b[1] * y + b[2] * z + b[3]) / depth;
    const [radial, tx, ty] = distort(nx, ny, D);
    const dx = nx * radial + tx;
    const dy = ny * radial + ty;
    const u = fx * dx + cx;
    const v = fy * dy + cy;
    if (!(u >= -0.5 && u < width - 0.5 && v >= -0.5 && v < height - 0.5)) continue;
    if (hasDistortion && !agreesWithInverseLens(dx, dy, nx, ny, D)) continue;
    const pixel = Math.floor(v + 0.5) * width + Math.floor(u + 0.5);
    projected.push({ u, v, depth, pixel, range: Math.fround(range) });
  }

  projected.sort((first, second) => first.pixel - second.pixel || first.depth - second.depth);
  const sparse = new Float32Array(width * height).fill(NaN);
  const offsets = new Uint32Array(sparse.length + 1);
  for (const point of projected) {
    if (offsets[point.pixel + 1]++ === 0) sparse[point.pixel] = point.range;
  }
  for (let i = 1; i < offsets.length; i++) offsets[i] += offsets[i - 1];
  return { width, height, projected, offsets, sparse };
}

function selectCluster(neighbours, jumpThreshold, minClusterPoints, ratioThreshold) {
  let start = 0;
  let foregroundEnd = 0;
  let foregroundStart = 0;
  let backgroundSize = 0;
  let backgroundStart = 0;
  for (let end = 1; end <= neighbours.length; end++) {
    if (end < neighbours.length) {
      const previous = neighbours[end - 1].range;
      const current = neighbours[end].range;
      if ((current - previous) / (current + previous) <= jumpThreshold) continue;
    }
    const size = end - start;
    if (size >= minClusterPoints) {
      if (!foregroundEnd) {
        foregroundStart = start;
        foregroundEnd = end;
      } else if (size > backgroundSize) {
        backgroundStart = start;
        backgroundSize = size;
      }
    }
    start = end;
  }
  if (!backgroundSize) return [0, neighbours.length];
  return (foregroundEnd - foregroundStart) / backgroundSize >= ratioThreshold
    ? [foregroundStart, foregroundEnd]
    : [backgroundStart, backgroundStart + backgroundSize];
}

function densify(frame, { windowSize, jumpThreshold, minClusterPoints, ratioThreshold }) {
  const { width, height, projected, offsets, sparse } = frame;
  const radius = Math.floor(windowSize / 2);
  const dense = new Float32Array(sparse.length).fill(NaN);
  const neighbours = [];
  const byRange = (first, second) => first.range - second.range;
  for (let y = 0; y < height; y++) {
    const top = Math.max(0, y - radius);
    const bottom = Math.min(height, y + radius + 1);
    for (let x = 0; x < width; x++) {
      const left = Math.max(0, x - radius);
      const right = Math.min(width, x + radius + 1);
      neighbours.length = 0;
      for (let row = top; row < bottom; row++) {
        const start = offsets[row * width + left];
        const end = offsets[row * width + right];
        for (let n = start; n < end; n++) neighbours.push(projected[n]);
      }
      if (!neighbours.length) continue;
      neighbours.sort(byRange);
      const [start, end] = selectCluster(neighbours, jumpThreshold, minClusterPoints, ratioThreshold);
      const pixel = y * width + x;
      const reference = Number.isNaN(sparse[pixel]) ? neighbours[0].range : sparse[pixel];
      let sum = 0;
      let weights = 0;
      for (let n = start; n < end; n++) {
        const point = neighbours[n];
        const dx = x - point.u;
        const dy = y - point.v;
        const spatialWeight = 1 / (1 + Math.sqrt(dx * dx + dy * dy));
        const rangeWeight = 1 / (1 + Math.abs(reference - point.range));
        const weight = spatialWeight * rangeWeight;
        sum += point.range * weight;
        weights += weight;
      }
      if (weights > 0) dense[pixel] = sum / weights;
    }
  }
  return dense;
}

let frame;
self.onmessage = ({ data }) => {
  try {
    const start = performance.now();
    if (data.type === "prepare") {
      frame = prepareFrame(data.ply, parseCalibration(data.calibration));
      const sparse = frame.sparse.slice();
      self.postMessage({ type: "sparse", width: frame.width, height: frame.height, count: frame.projected.length, sparse }, [sparse.buffer]);
    } else if (data.type === "densify" && frame) {
      const dense = densify(frame, data.settings);
      self.postMessage({ type: "dense", revision: data.revision, elapsed: performance.now() - start, dense }, [dense.buffer]);
    }
  } catch (error) {
    self.postMessage({ type: "error", message: error.message });
  }
};
