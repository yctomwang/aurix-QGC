// Synthetic point cloud generator used for the Fly View point cloud POC.
// Produces a low-density "hills" surface with a bit of noise so the
// visualization has depth without pulling data from an external source.

var _emptyResult = { points: [], dronePos: null };

function makeDemoPointCloud(rows, cols) {
    const points = [];
    const scale = 0.6;

    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            const x = (i - rows / 2) * scale;
            const y = (j - cols / 2) * scale;
            const r = Math.sqrt(x * x + y * y);

            // Height follows a gentle hill profile with a soft ripple so it
            // stays interesting when rotated.
            const z = 4
                + 1.8 * Math.sin(i * 0.25)
                + 1.2 * Math.cos(j * 0.33)
                - 0.25 * r * r / rows;

            // Map height to a 0-1 intensity used for coloring.
            const intensity = Math.max(0.05, Math.min(1, (z + 2) / 8));
            points.push({ x, y, z, intensity });
        }
    }
    return points;
}

function randomNoise(points, magnitude) {
    return points.map(p => ({
        x: p.x + (Math.random() - 0.5) * magnitude,
        y: p.y + (Math.random() - 0.5) * magnitude,
        z: p.z + (Math.random() - 0.5) * magnitude * 0.35,
        intensity: p.intensity,
    }));
}

function demoCloud() {
    const grid = makeDemoPointCloud(22, 22);
    return randomNoise(grid, 0.6);
}

function parseCSV(csvText) {
    const lines = csvText.split('\n');
    const points = [];
    // Skip header if present (check for "x,y,z")
    let start = 0;
    if (lines.length > 0 && lines[0].indexOf('x') !== -1) {
        start = 1;
    }
    
    // Limit to reasonable number for Canvas performance if file is huge
    const limit = 50000;
    
    for (let i = start; i < lines.length; i++) {
        if (points.length >= limit) break;
        
        const line = lines[i].trim();
        if (line.length === 0) continue;
        
        const parts = line.split(',');
        if (parts.length >= 3) {
            const x = parseFloat(parts[0]);
            const y = parseFloat(parts[1]);
            const z = parseFloat(parts[2]);
            let intensity = 0.5;
            if (parts.length > 3) {
                intensity = parseFloat(parts[3]);
            }
            if (!isNaN(x) && !isNaN(y) && !isNaN(z)) {
                points.push({x, y, z, intensity});
            }
        }
    }
    return points;
}

// Apply axis permutation so display matches companion frame. order: "xyz" (default), "xzy", "yxz", "yzx", "zxy", "zyx".
function applyAxisOrder(points, order) {
    if (!points || points.length === 0 || !order || order === "xyz") return points;
    var o = String(order).toLowerCase();
    var out = [];
    for (var i = 0; i < points.length; i++) {
        var p = points[i];
        var a = p.x, b = p.y, c = p.z;
        var nx, ny, nz;
        if (o === "xzy") { nx = a; ny = c; nz = b; }
        else if (o === "yxz") { nx = b; ny = a; nz = c; }
        else if (o === "yzx") { nx = b; ny = c; nz = a; }
        else if (o === "zxy") { nx = c; ny = a; nz = b; }
        else if (o === "zyx") { nx = c; ny = b; nz = a; }
        else { nx = a; ny = b; nz = c; }
        out.push({ x: nx, y: ny, z: nz, intensity: p.intensity !== undefined ? p.intensity : 0.5 });
    }
    return out;
}

// Compute bounding box and extent for auto-fit view. Returns { centerX, centerY, centerZ, extent, minX, maxX, minY, maxY, minZ, maxZ }.
function computeBounds(points) {
    if (!points || points.length === 0)
        return { centerX: 0, centerY: 0, centerZ: 0, extent: 20, minX: -10, maxX: 10, minY: -10, maxY: 10, minZ: -10, maxZ: 10 };
    var minX = Infinity, maxX = -Infinity;
    var minY = Infinity, maxY = -Infinity;
    var minZ = Infinity, maxZ = -Infinity;
    for (var i = 0; i < points.length; i++) {
        var p = points[i];
        if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) continue;
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
        if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
    }
    if (minX === Infinity || !isFinite(minX)) {
        return { centerX: 0, centerY: 0, centerZ: 0, extent: 20, minX: -10, maxX: 10, minY: -10, maxY: 10, minZ: -10, maxZ: 10 };
    }
    var centerX = (minX + maxX) / 2;
    var centerY = (minY + maxY) / 2;
    var centerZ = (minZ + maxZ) / 2;
    var extent = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1);
    return { centerX: centerX, centerY: centerY, centerZ: centerZ, extent: extent, minX: minX, maxX: maxX, minY: minY, maxY: maxY, minZ: minZ, maxZ: maxZ };
}

// Optional: set to true to use Big Endian (some devices use BE)
var _binaryBigEndian = false;
function setBinaryBigEndian(useBE) { _binaryBigEndian = useBE; }
function getBinaryBigEndian() { return _binaryBigEndian; }

// 16 B/pt (x,y,z,intensity) is the standard companion format. Set true only for legacy 12 B/pt servers.
var _binary12Bytes = false;
function setBinary12Bytes(use12) { _binary12Bytes = use12; }

// Optional: 16 bytes per point but x,y,z at offsets 4,8,12 (legacy; use standard x,y,z,i when possible).
var _binary16Skip4 = false;
function setBinary16Skip4(skip4) { _binary16Skip4 = skip4; }

// Optional: if true, first 4 bytes are int32 LE = point count, then count × 16 bytes of points (SEND_COUNT_HEADER).
var _binaryCountHeader = false;
function setBinaryCountHeader(useHeader) { _binaryCountHeader = useHeader; }

// Parse from raw bytes (ArrayBuffer or Uint8Array). Same format: 16 B (x,y,z,i) or 12 B, optional count header.
// Returns { points: [], dronePos: {x,y,z,yaw}|null }.
function parseRawBinary(arrayBufferOrBytes, debugFirstPoint) {
    var bytes;
    if (arrayBufferOrBytes instanceof ArrayBuffer) {
        bytes = new Uint8Array(arrayBufferOrBytes);
    } else if (arrayBufferOrBytes instanceof Uint8Array) {
        bytes = arrayBufferOrBytes;
    } else {
        console.log("[PointCloud] parseRawBinary: expected ArrayBuffer or Uint8Array");
        return _emptyResult;
    }
    return parseBinaryBytes(bytes, bytes.length, debugFirstPoint);
}

// Base64 decode to raw bytes. Strips whitespace/non-Base64; uses low 8 bits per char (robust to encoding).
function base64DecodeToBytes(base64Str) {
    var raw = String(base64Str);
    var cleaned = "";
    for (var i = 0; i < raw.length; i++) {
        var c = raw.charCodeAt(i);
        var ch = raw.charAt(i);
        if ((ch >= "A" && ch <= "Z") || (ch >= "a" && ch <= "z") || (ch >= "0" && ch <= "9") || ch === "+" || ch === "/" || ch === "=")
            cleaned += ch;
    }
    var binaryString = "";
    try {
        if (typeof Qt !== "undefined" && Qt.atob)
            binaryString = Qt.atob(cleaned);
        else
            binaryString = atob(cleaned);
    } catch (e) {
        try { binaryString = atob(cleaned); } catch (e2) {}
        if (!binaryString) {
            console.log("[PointCloud] Base64 decode failed");
            return null;
        }
    }
    var len = binaryString.length;
    var bytes = new Uint8Array(len);
    for (var i = 0; i < len; i++)
        bytes[i] = binaryString.charCodeAt(i) & 0xFF;
    return bytes;
}

// Convert a string to bytes (one byte per character, low 8 bits). For raw binary delivered as text.
function stringToBytes(str) {
    var len = str.length;
    var bytes = new Uint8Array(len);
    for (var i = 0; i < len; i++)
        bytes[i] = str.charCodeAt(i) & 0xFF;
    return bytes;
}

// Heuristic: decoded cloud is "sane" if extent and coordinates are in a reasonable range (meters).
// Accepts either an array of points or { points: [], dronePos: ... }.
function isSaneCloud(result) {
    var pts = Array.isArray(result) ? result : (result ? result.points : null);
    if (!pts || pts.length === 0) return false;
    var b = computeBounds(pts);
    if (!isFinite(b.extent) || b.extent <= 0 || b.extent > 1e6) return false;
    for (var i = 0; i < Math.min(pts.length, 50); i++) {
        var p = pts[i];
        if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) return false;
        if (Math.abs(p.x) > 1e6 || Math.abs(p.y) > 1e6 || Math.abs(p.z) > 1e6) return false;
    }
    return true;
}

// Try Base64 first; if result is garbage (insane extent/coords), try raw binary (message = bytes as text).
// Returns { points: [], dronePos: {x,y,z,yaw}|null }.
function parseBase64Binary(base64Data, debugFirstPoint) {
    var msg = String(base64Data);
    var bytes = base64DecodeToBytes(msg);
    if (bytes && bytes.length > 0) {
        var result = parseBinaryBytes(bytes, bytes.length, debugFirstPoint);
        if (isSaneCloud(result))
            return result;
    }
    if (msg.length >= 12 && (msg.length % 12 === 0 || msg.length % 16 === 0)) {
        var rawBytes = stringToBytes(msg);
        var result = parseBinaryBytes(rawBytes, rawBytes.length, debugFirstPoint);
        if (isSaneCloud(result)) {
            if (debugFirstPoint && result.points.length > 0)
                console.log("[PointCloud] Using raw-binary-as-text (message length " + msg.length + " bytes)");
            return result;
        }
    }
    return _emptyResult;
}

// First 16 bytes of each frame = drone position (4× float32 LE: x, y, z, yaw).
var _hasDroneHeader = true;
function setHasDroneHeader(v) { _hasDroneHeader = v; }

// bytes = Uint8Array, len = number of bytes to use. Returns { points: [], dronePos: null }.
function parseBinaryBytes(bytes, len, debugFirstPoint) {
    var buf = bytes.buffer;
    var off = bytes.byteOffset || 0;
    var dataView = new DataView(buf, off, len);
    var pointSize = _binary12Bytes ? 12 : 16;
    var dataStart = 0;
    var dronePos = null;

    // Drone position header: first 16 bytes = x, y, z, yaw (float32 LE)
    if (_hasDroneHeader && len >= 16) {
        var le = !_binaryBigEndian;
        var dx = dataView.getFloat32(0, le);
        var dy = dataView.getFloat32(4, le);
        var dz = dataView.getFloat32(8, le);
        var dyaw = dataView.getFloat32(12, le);
        if (isFinite(dx) && isFinite(dy) && isFinite(dz)) {
            dronePos = { x: dx, y: dy, z: dz, yaw: isFinite(dyaw) ? dyaw : 0 };
        }
        dataStart = 16;
        if (debugFirstPoint && dronePos)
            console.log("[PointCloud] Drone pos: x=" + dx.toFixed(2) + " y=" + dy.toFixed(2) + " z=" + dz.toFixed(2) + " yaw=" + dyaw.toFixed(2));
    }

    var count;
    if (_binaryCountHeader && len >= dataStart + 4 && !_binary12Bytes) {
        count = dataView.getInt32(dataStart, true);
        dataStart += 4;
        var expectedLen = dataStart + count * 16;
        if (count < 0 || count > 1000000 || expectedLen > len) {
            count = Math.floor((len - dataStart) / 16);
        }
    } else {
        count = Math.floor((len - dataStart) / pointSize);
    }

    var points = [];
    var littleEndian = !_binaryBigEndian;
    var skip4 = !_binary12Bytes && _binary16Skip4;

    function readPoint(offset, le) {
        var x, y, z, intensity;
        if (_binary12Bytes) {
            x = dataView.getFloat32(offset, le);
            y = dataView.getFloat32(offset + 4, le);
            z = dataView.getFloat32(offset + 8, le);
            intensity = 0.5;
        } else if (skip4) {
            x = dataView.getFloat32(offset + 4, le);
            y = dataView.getFloat32(offset + 8, le);
            z = dataView.getFloat32(offset + 12, le);
            intensity = 0.5;
        } else {
            x = dataView.getFloat32(offset, le);
            y = dataView.getFloat32(offset + 4, le);
            z = dataView.getFloat32(offset + 8, le);
            intensity = dataView.getFloat32(offset + 12, le);
        }
        return { x: x, y: y, z: z, intensity: intensity };
    }

    for (var i = 0; i < count; i++) {
        var offset = dataStart + i * pointSize;
        var p = readPoint(offset, littleEndian);
        if (isFinite(p.x) && isFinite(p.y) && isFinite(p.z))
            points.push(p);
    }

    if (debugFirstPoint && count > 0) {
        var firstOffset = dataStart;
        var hex = "";
        for (var j = 0; j < pointSize && (firstOffset + j) < len; j++) {
            var b = bytes[firstOffset + j];
            hex += (b < 16 ? "0" : "") + b.toString(16) + " ";
        }
        var leP = readPoint(firstOffset, true);
        var beP = readPoint(firstOffset, false);
        console.log("[PointCloud] First point raw hex (12 B): " + hex);
        console.log("[PointCloud] Decoded LE x,y,z: " + leP.x.toFixed(4) + ", " + leP.y.toFixed(4) + ", " + leP.z.toFixed(4));
        console.log("[PointCloud] Decoded BE x,y,z: " + beP.x.toFixed(4) + ", " + beP.y.toFixed(4) + ", " + beP.z.toFixed(4));
        var layout = _binary12Bytes ? "12 B (x,y,z) struct.pack('<fff')" : (skip4 ? "16 B skip 4" : "16 B (x,y,z,i)");
        if (_binaryCountHeader && !_binary12Bytes) layout += " + count header";
        console.log("[PointCloud] " + layout + ", " + (_binaryBigEndian ? "BE" : "LE") + ", " + count + " pts, " + len + " B");
    }

    return { points: points, dronePos: dronePos };
}
