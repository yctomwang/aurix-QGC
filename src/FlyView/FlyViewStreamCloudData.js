// FlyViewStreamCloudData.js
// Data layer for the stream_server.py WebSocket protocol.
// Handles: type 0x01 (zlib-compressed XYZRGB point cloud) and type 0x02 (odometry).
// Completely separate from FlyViewPointCloudData.js (legacy protocol).

var MSG_POINTCLOUD = 0x01;
var MSG_ODOMETRY   = 0x02;

// Reference to Zlib.zlibInflate, injected from QML via setZlibInflate().
var _zlibInflate = null;
function setZlibInflate(fn) { _zlibInflate = fn; }

// ── Bounds / center helpers ──────────────────────────────────────────────────

function computeBounds(points) {
    if (!points || points.length === 0)
        return { centerX: 0, centerY: 0, centerZ: 0, extent: 20,
                 minX: -10, maxX: 10, minY: -10, maxY: 10, minZ: -10, maxZ: 10 };
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
    if (!isFinite(minX))
        return { centerX: 0, centerY: 0, centerZ: 0, extent: 20,
                 minX: -10, maxX: 10, minY: -10, maxY: 10, minZ: -10, maxZ: 10 };
    return {
        centerX: (minX + maxX) / 2,
        centerY: (minY + maxY) / 2,
        centerZ: (minZ + maxZ) / 2,
        extent:  Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1),
        minX: minX, maxX: maxX,
        minY: minY, maxY: maxY,
        minZ: minZ, maxZ: maxZ
    };
}

// ── Type 0x01 — Point Cloud ──────────────────────────────────────────────────
// Frame: byte 0 = 0x01, bytes 1..N = zlib-compressed payload.
// Decompressed payload: uint32 LE point count, then N × 16 bytes (x,y,z,rgb_packed).
// Returns { points: [{x,y,z,r,g,b,intensity}], count } or null.

function parseStreamPointCloud(frameBytes, debugFirstPoint) {
    if (!frameBytes || frameBytes.length < 2) return null;
    if (!_zlibInflate) {
        console.log("[StreamCloud] zlibInflate not set — cannot decompress");
        return null;
    }

    var compressed = frameBytes.subarray(1);
    var decompressed;
    try {
        decompressed = _zlibInflate(compressed);
    } catch (e) {
        console.log("[StreamCloud] zlib inflate error: " + e);
        return null;
    }
    if (!decompressed || decompressed.length < 4) return null;

    var dv    = new DataView(decompressed.buffer, decompressed.byteOffset, decompressed.length);
    var count = dv.getUint32(0, true);
    if (decompressed.length < 4 + count * 16)
        count = Math.floor((decompressed.length - 4) / 16);

    // Shared typed-array trick for packed-float-to-uint32 reinterpretation.
    var tempU32 = new Uint32Array(1);
    var tempF32 = new Float32Array(tempU32.buffer);

    var points = [];
    for (var i = 0; i < count; i++) {
        var off = 4 + i * 16;
        var rosX = dv.getFloat32(off,      true);
        var rosY = dv.getFloat32(off + 4,  true);
        var rosZ = dv.getFloat32(off + 8,  true);

        if (!isFinite(rosX) || !isFinite(rosY) || !isFinite(rosZ)) continue;

        // Decode packed RGB: 4th float reinterpreted as uint32.
        // bits 23-16 = R, 15-8 = G, 7-0 = B
        tempF32[0] = dv.getFloat32(off + 12, true);
        var rgb = tempU32[0];
        var r = (rgb >>> 16) & 0xFF;
        var g = (rgb >>>  8) & 0xFF;
        var b =  rgb         & 0xFF;

        // ROS frame (X-forward, Y-left, Z-up) → display canvas (X-right, Y-forward, Z-up)
        points.push({
            x:         -rosY,
            y:          rosX,
            z:          rosZ,
            r: r, g: g, b: b,
            intensity: (r * 0.299 + g * 0.587 + b * 0.114) / 255.0
        });
    }

    if (debugFirstPoint && points.length > 0) {
        var p0 = points[0];
        console.log("[StreamCloud] 0x01: " + count + " pts compressed=" + compressed.length
                    + "B → " + decompressed.length + "B, first=("
                    + p0.x.toFixed(3) + "," + p0.y.toFixed(3) + "," + p0.z.toFixed(3)
                    + ") RGB(" + p0.r + "," + p0.g + "," + p0.b + ")");
    }
    return { points: points, count: count };
}

// ── Type 0x02 — Odometry ────────────────────────────────────────────────────
// Frame: byte 0 = 0x02, bytes 1-28 = 7 × float32 LE (x,y,z, qx,qy,qz,qw).
// Returns { x, y, z, qx, qy, qz, qw, yaw } or null.

function parseStreamOdometry(frameBytes, debugFirstPoint) {
    if (!frameBytes || frameBytes.length < 29) return null;

    var dv = new DataView(frameBytes.buffer, frameBytes.byteOffset, frameBytes.length);
    var rosX = dv.getFloat32(1,  true);
    var rosY = dv.getFloat32(5,  true);
    var rosZ = dv.getFloat32(9,  true);
    var qx   = dv.getFloat32(13, true);
    var qy   = dv.getFloat32(17, true);
    var qz   = dv.getFloat32(21, true);
    var qw   = dv.getFloat32(25, true);

    if (!isFinite(rosX) || !isFinite(rosY) || !isFinite(rosZ)) return null;

    // Quaternion → yaw (heading) in degrees
    var yaw = Math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / Math.PI;

    if (debugFirstPoint)
        console.log("[StreamCloud] 0x02 Odom: ros=(" + rosX.toFixed(2) + ","
                    + rosY.toFixed(2) + "," + rosZ.toFixed(2) + ") yaw=" + yaw.toFixed(1) + "°");

    return { x: -rosY, y: rosX, z: rosZ, qx: qx, qy: qy, qz: qz, qw: qw, yaw: yaw };
}

// ── Dispatcher ───────────────────────────────────────────────────────────────
// Returns { type: "pointcloud"|"odometry"|"unknown", data: ... }

function parseStreamMessage(arrayBuffer, debugFirstPoint) {
    var bytes;
    if (arrayBuffer instanceof ArrayBuffer)
        bytes = new Uint8Array(arrayBuffer);
    else if (arrayBuffer instanceof Uint8Array)
        bytes = arrayBuffer;
    else
        return { type: "unknown", data: null };

    if (bytes.length < 1) return { type: "unknown", data: null };

    var msgType = bytes[0];
    if (msgType === MSG_POINTCLOUD) {
        return { type: "pointcloud", data: parseStreamPointCloud(bytes, debugFirstPoint) };
    } else if (msgType === MSG_ODOMETRY) {
        return { type: "odometry", data: parseStreamOdometry(bytes, debugFirstPoint) };
    }
    return { type: "unknown", data: null };
}
