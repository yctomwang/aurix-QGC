import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QGroundControl
import QGroundControl.Controls
import QtWebSockets
import "FlyViewStreamCloudData.js" as StreamData
import "FlyViewZlib.js" as Zlib

Rectangle {
    id: root
    color: "#000000"

    // ── Connection ──────────────────────────────────────────────────────────
    property string companionIp: "192.168.144.100"
    property int    wsPort:      9090

    // ── Camera ──────────────────────────────────────────────────────────────
    property real camPitch: -0.5   // negative = look DOWN (correct for drone top-view)
    property real camYaw:   0.0
    property real camZoom:  30.0
    property bool negateZ:  true   // Z-up display

    // ── Interaction state (used for LOD) ─────────────────────────────────────
    property bool isInteracting: false

    // ── Scene state ─────────────────────────────────────────────────────────
    property string title:         "Stream Cloud"
    property var    dronePos:      null
    property var    selectedPoint: null
    property real   distToTarget:  0.0

    property real cloudCenterX: 0
    property real cloudCenterY: 0
    property real cloudCenterZ: 0
    property real cloudExtent:  20
    property var  cloudBounds:  null
    property bool viewLocked:   false   // when true, incoming data won't move the camera

    // ── Telemetry ────────────────────────────────────────────────────────────
    property real telHeading:  0.0
    property real telAltitude: 0.0

    // ── Map accumulation ─────────────────────────────────────────────────────
    property bool mapMode:       true
    property int  totalMapPoints: 0
    property real mapVoxelSize:   0.1    // matches server-side 0.1 m voxel dedup
    property int  ingestSampleRate: 1    // server already downsampled; keep all
    property int  mapPointCap:    1000000

    property int  renderPointCap: 20000

    // Flat storage: 6 floats per point [x,y,z,r,g,b, x,y,z,r,g,b, ...]
    property var  _mapFlat:     []       // flat array: x,y,z,r,g,b per point
    property int  _mapCount:    0
    property var  _mapVoxels:   ({})
    property var  _renderFlat:  []       // pre-downsampled flat array for paint loop
    property int  _renderCount: 0
    property int  _renderBufDirty: 0
    property real _mapMinX:  Infinity;  property real _mapMaxX: -Infinity
    property real _mapMinY:  Infinity;  property real _mapMaxY: -Infinity
    property real _mapMinZ:  Infinity;  property real _mapMaxZ: -Infinity

    // ── Internal ─────────────────────────────────────────────────────────────
    property int  _frameCount:  0
    property bool _paintDirty: false

    // ── Visual constants ─────────────────────────────────────────────────────
    readonly property color colorAccent:   "#00CED1"
    readonly property color colorGrid:     "#333333"
    readonly property color colorWaypoint: "#FFD700"

    // ── Waypoints ────────────────────────────────────────────────────────────
    property var waypoints:          []
    property int nextWaypointId:     1
    property int selectedWaypointIdx: -1
    property int draggingWaypointIdx: -1
    property var _wpDragStart:       null

    // ── Exploration box ──────────────────────────────────────────────────────
    property string boxState:      "idle"
    property var    explorationBox: null
    property int    activeHandleIdx: -1
    property point  _dragStartMouse: Qt.point(0, 0)
    property var    _dragStartBox:   null
    property string activeTool:    "orbit"
    readonly property real boxMinSize: 5.0
    readonly property real boxMaxSize: 500.0
    readonly property color colorBoxEditing:   "#B8B800"
    readonly property color colorBoxHovering:  "#FFD700"
    readonly property color colorBoxConfirmed: "#32CD32"
    readonly property color colorBoxInvalid:   "#FF4500"

    signal boxConfirmed(var boxData)

    // ── Init ─────────────────────────────────────────────────────────────────
    Component.onCompleted: {
        StreamData.setZlibInflate(Zlib.zlibInflate)
    }

    // ── Paint throttle ───────────────────────────────────────────────────────
    // 8 fps at rest (plenty for point cloud), 15 fps during interaction
    Timer {
        interval: root.isInteracting ? 66 : 120
        repeat: true; running: true
        onTriggered: { if (root._paintDirty) { root._paintDirty = false; canvas.requestPaint(); } }
    }

    // ── Interaction-end timer: restore full quality after last drag/wheel ──
    Timer {
        id: interactionEndTimer
        interval: 300; repeat: false
        onTriggered: { root.isInteracting = false; root._paintDirty = true; }
    }
    function _startInteraction() {
        isInteracting = true;
        viewLocked = true;
        interactionEndTimer.restart();
        _paintDirty = true;
    }

    // ── Map helpers ──────────────────────────────────────────────────────────
    function clearMap() {
        _mapFlat = []; _mapCount = 0; _mapVoxels = ({});
        _renderFlat = []; _renderCount = 0;
        _mapMinX =  Infinity; _mapMaxX = -Infinity;
        _mapMinY =  Infinity; _mapMaxY = -Infinity;
        _mapMinZ =  Infinity; _mapMaxZ = -Infinity;
        totalMapPoints = 0;
        root._paintDirty = true;
    }

    function _rebuildRenderBuf() {
        var cnt = _mapCount, cap = renderPointCap;
        var src = _mapFlat;
        if (cnt <= cap) {
            _renderFlat = src;
            _renderCount = cnt;
            return;
        }
        var step = cnt / cap;
        var buf = new Array(cap * 6);
        for (var i = 0; i < cap; i++) {
            var si = ((i * step) | 0) * 6;
            var di = i * 6;
            buf[di]   = src[si];   buf[di+1] = src[si+1]; buf[di+2] = src[si+2];
            buf[di+3] = src[si+3]; buf[di+4] = src[si+4]; buf[di+5] = src[si+5];
        }
        _renderFlat = buf;
        _renderCount = cap;
    }

    // Evict oldest points to make room, clearing their voxel keys
    function _evictOldest(evictCount) {
        var invRes = 1.0 / mapVoxelSize;
        var flat = _mapFlat, voxels = _mapVoxels;
        var evict6 = evictCount * 6;
        for (var i = 0; i < evict6; i += 6) {
            var ix = (flat[i]   * invRes + 0.5) | 0;
            var iy = (flat[i+1] * invRes + 0.5) | 0;
            var iz = (flat[i+2] * invRes + 0.5) | 0;
            delete voxels[ix + "," + iy + "," + iz];
        }
        _mapFlat = flat.slice(evict6);
        _mapCount -= evictCount;
    }

    function mergeIntoMap(newPts) {
        if (!newPts || newPts.length === 0) return 0;
        var invRes = 1.0 / mapVoxelSize;
        var step   = Math.max(1, ingestSampleRate);
        var voxels = _mapVoxels, flat = _mapFlat, cnt = _mapCount, added = 0;
        var minX = _mapMinX, maxX = _mapMaxX;
        var minY = _mapMinY, maxY = _mapMaxY;
        var minZ = _mapMinZ, maxZ = _mapMaxZ;
        for (var i = 0; i < newPts.length; i += step) {
            var p = newPts[i];
            var ix = (p.x * invRes + 0.5) | 0;
            var iy = (p.y * invRes + 0.5) | 0;
            var iz = (p.z * invRes + 0.5) | 0;
            var key = ix + "," + iy + "," + iz;
            if (voxels[key]) continue;
            voxels[key] = true;
            flat.push(p.x, p.y, p.z, p.r || 128, p.g || 128, p.b || 128);
            cnt++; added++;
            if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
            if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
        }
        if (added > 0) {
            _mapCount = cnt;
            _mapMinX = minX; _mapMaxX = maxX;
            _mapMinY = minY; _mapMaxY = maxY;
            _mapMinZ = minZ; _mapMaxZ = maxZ;
            totalMapPoints = cnt;

            // Rolling window: evict oldest 20% when cap is hit
            if (cnt > mapPointCap) {
                var evict = Math.floor(mapPointCap * 0.2);
                _evictOldest(evict);
                totalMapPoints = _mapCount;
            }

            _renderBufDirty += added;
            var rebuildThreshold = _mapCount < renderPointCap ? 50 : 2000;
            if (_renderBufDirty >= rebuildThreshold) {
                _renderBufDirty = 0;
                _rebuildRenderBuf();
            }
        }
        return added;
    }

    function applyMapBounds() {
        if (!isFinite(_mapMinX) || _mapCount === 0) return;
        if (viewLocked) return;
        cloudCenterX = (_mapMinX + _mapMaxX) * 0.5;
        cloudCenterY = (_mapMinY + _mapMaxY) * 0.5;
        cloudCenterZ = (_mapMinZ + _mapMaxZ) * 0.5;
        cloudExtent  = Math.max(_mapMaxX - _mapMinX,
                                _mapMaxY - _mapMinY,
                                _mapMaxZ - _mapMinZ, 1.0);
    }

    function fitAll() {
        if (!isFinite(_mapMinX) || _mapCount === 0) return;
        cloudCenterX = (_mapMinX + _mapMaxX) * 0.5;
        cloudCenterY = (_mapMinY + _mapMaxY) * 0.5;
        cloudCenterZ = (_mapMinZ + _mapMaxZ) * 0.5;
        cloudExtent  = Math.max(_mapMaxX - _mapMinX,
                                _mapMaxY - _mapMinY,
                                _mapMaxZ - _mapMinZ, 1.0);
        camZoom = 30; viewLocked = false; _paintDirty = true;
    }

    function centerOnDrone() {
        if (!dronePos) return;
        cloudCenterX = dronePos.x;
        cloudCenterY = dronePos.y;
        cloudCenterZ = dronePos.z;
        cloudExtent  = 20;
        viewLocked = true; _paintDirty = true;
    }

    // ── Box helpers ──────────────────────────────────────────────────────────
    function _boxColor() {
        switch (boxState) {
        case "hovering": case "dragging": return colorBoxHovering;
        case "confirmed": return colorBoxConfirmed;
        case "invalid":   return colorBoxInvalid;
        default:          return colorBoxEditing;
        }
    }
    function _boxOpacity() {
        switch (boxState) {
        case "hovering":  return 0.45;
        case "dragging": case "invalid": return 0.50;
        case "confirmed": return 0.25;
        default: return 0.35;
        }
    }
    function createExplorationBox() {
        var cx = cloudCenterX, cy = cloudCenterY, cz = cloudCenterZ;
        if (dronePos) { cx = dronePos.x; cy = dronePos.y; cz = dronePos.z; }
        explorationBox = { cx: cx, cy: cy, cz: cz, dx: 5, dy: 5, dz: 2.5 };
        boxState = "editing"; root._paintDirty = true;
    }
    function validateBox() {
        if (!explorationBox) return false;
        var b = explorationBox;
        var w = b.dx*2, d = b.dy*2, h = b.dz*2;
        return w >= boxMinSize && d >= boxMinSize && h >= boxMinSize*0.6
            && w <= boxMaxSize && d <= boxMaxSize && h <= boxMaxSize*0.2;
    }
    function confirmBox() {
        if (!explorationBox) return;
        if (!validateBox()) { boxState = "invalid"; root._paintDirty = true; return; }
        boxState = "confirmed";
        var b = explorationBox;
        boxConfirmed({ position: [b.cx,b.cy,b.cz], dimensions: [b.dx*2,b.dy*2,b.dz*2] });
        root._paintDirty = true;
    }
    function deleteBox() {
        explorationBox = null; boxState = "idle"; activeHandleIdx = -1;
        root._paintDirty = true;
    }

    // ── Waypoint helpers ─────────────────────────────────────────────────────
    function addWaypoint(wx, wy, wz) {
        var wp = { id: nextWaypointId, x: wx, y: wy, z: wz };
        var a = []; for (var i = 0; i < waypoints.length; i++) a.push(waypoints[i]);
        a.push(wp); waypoints = a; nextWaypointId++;
        selectedWaypointIdx = a.length - 1; root._paintDirty = true;
    }
    function updateWaypoint(idx, wx, wy, wz) {
        if (idx < 0 || idx >= waypoints.length) return;
        var a = []; for (var i = 0; i < waypoints.length; i++)
            a.push(i === idx ? { id: waypoints[i].id, x: wx, y: wy, z: wz } : waypoints[i]);
        waypoints = a; root._paintDirty = true;
    }
    function deleteWaypoint(idx) {
        if (idx < 0 || idx >= waypoints.length) return;
        var a = []; for (var i = 0; i < waypoints.length; i++) if (i !== idx) a.push(waypoints[i]);
        waypoints = a;
        if (selectedWaypointIdx === idx) selectedWaypointIdx = -1;
        else if (selectedWaypointIdx > idx) selectedWaypointIdx--;
        root._paintDirty = true;
    }

    // ── 3D ↔ 2D projection ───────────────────────────────────────────────────
    function project3D(wx, wy, wz, cxC, cyC, scale, cosY, sinY, cosP, sinP) {
        var x0 = wx - cloudCenterX, y0 = wy - cloudCenterY, z0 = wz - cloudCenterZ;
        var z0v = negateZ ? -z0 : z0;
        var x1 = x0*cosY - y0*sinY, y1 = x0*sinY + y0*cosY;
        var x2 = x1, y2 = y1*cosP - z0v*sinP;
        return { sx: cxC + x2*scale, sy: cyC - y2*scale };
    }
    function unproject2D(screenX, screenY, targetZ) {
        var extent = Math.max(cloudExtent, 1);
        var scale  = Math.min(canvas.width, canvas.height) / (extent*1.2) * (camZoom/30);
        if (scale === 0) return { x: cloudCenterX, y: cloudCenterY, z: targetZ };
        var cxC = canvas.width/2, cyC = canvas.height/2;
        var x2 = (screenX - cxC) / scale, y2 = -(screenY - cyC) / scale;
        var cosP = Math.cos(camPitch), sinP = Math.sin(camPitch);
        var cosY = Math.cos(camYaw),   sinY = Math.sin(camYaw);
        var z0 = targetZ - cloudCenterZ;
        var z1 = negateZ ? -z0 : z0;
        var y1 = Math.abs(cosP) > 0.001 ? (y2 + z1*sinP) / cosP : y2;
        var x0 = x2*cosY + y1*sinY, y0 = -x2*sinY + y1*cosY;
        return { x: x0 + cloudCenterX, y: y0 + cloudCenterY, z: targetZ };
    }
    function hitTestWaypoint(sx, sy) {
        var extent = Math.max(cloudExtent, 1);
        var scale  = Math.min(canvas.width, canvas.height) / (extent*1.2) * (camZoom/30);
        var cxC = canvas.width/2, cyC = canvas.height/2;
        var cosY = Math.cos(camYaw), sinY = Math.sin(camYaw);
        var cosP = Math.cos(camPitch), sinP = Math.sin(camPitch);
        var best = -1, bestD = 18;
        for (var i = 0; i < waypoints.length; i++) {
            var w = waypoints[i];
            var p = project3D(w.x, w.y, w.z, cxC, cyC, scale, cosY, sinY, cosP, sinP);
            var d = Math.sqrt((p.sx-sx)*(p.sx-sx)+(p.sy-sy)*(p.sy-sy));
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    // ── WebSocket ─────────────────────────────────────────────────────────────
    WebSocket {
        id: socket
        url: "ws://" + root.companionIp + ":" + root.wsPort
        active: false

        function applyPointCloud(pts) {
            if (!pts || pts.length === 0) return;
            root._frameCount++;
            if (root.mapMode) {
                var added = root.mergeIntoMap(pts);
                if (added > 0) {
                    root.applyMapBounds();
                    root._paintDirty = true;
                }
                root.title = "Map: " + root.totalMapPoints + " pts  +" + added + " new";
            } else {
                var b = StreamData.computeBounds(pts);
                root.cloudBounds  = b;
                root.cloudCenterX = b.centerX; root.cloudCenterY = b.centerY;
                root.cloudCenterZ = b.centerZ; root.cloudExtent  = b.extent;
                var lf = new Array(pts.length * 6);
                for (var i = 0; i < pts.length; i++) {
                    var p = pts[i], j = i * 6;
                    lf[j] = p.x; lf[j+1] = p.y; lf[j+2] = p.z;
                    lf[j+3] = p.r || 128; lf[j+4] = p.g || 128; lf[j+5] = p.b || 128;
                }
                root._renderFlat = lf;
                root._renderCount = pts.length;
                root._paintDirty  = true;
                root.title = "Live: " + pts.length + " pts";
            }
        }

        onBinaryMessageReceived: function(message) {
            var msg = StreamData.parseStreamMessage(message, root._frameCount < 2);

            if (msg.type === "pointcloud" && msg.data)
                applyPointCloud(msg.data.points);
            else if (msg.type === "odometry" && msg.data) {
                var od = msg.data;
                root.dronePos    = { x: od.x, y: od.y, z: od.z, yaw: od.yaw };
                root.telHeading  = od.yaw;
                root.telAltitude = od.z;
                // Don't set _paintDirty here — odometry piggybacks on the next
                // point cloud repaint or the timer will pick it up naturally.
            }

            root._frameCount++;
        }

        onStatusChanged: {
            if (socket.status === WebSocket.Open) {
                console.log("[StreamCloud] Connected to " + socket.url);
                root._frameCount = 0;
            } else if (socket.status === WebSocket.Error) {
                console.log("[StreamCloud] Error: " + socket.errorString);
            } else if (socket.status === WebSocket.Closed) {
                console.log("[StreamCloud] Disconnected");
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // UI
    // ══════════════════════════════════════════════════════════════════════════
    Item {
        anchors.fill: parent

        // ── Left sidebar (tool palette) ───────────────────────────────────────
        Rectangle {
            id: leftSidebar
            anchors { left: parent.left; top: parent.top; bottom: statusBar.top }
            width: 48; color: "#1A1A1A"; opacity: 0.92; z: 10

            Column {
                anchors.centerIn: parent; spacing: 8
                Repeater {
                    model: [
                        { icon: "\u2316", tool: "orbit",    tip: "Orbit Camera" },
                        { icon: "\u25A1", tool: "box",      tip: "Exploration Box" },
                        { icon: "\u2316", tool: "waypoint", tip: "Add Waypoint" },
                        { icon: "\u25CE", tool: "select",   tip: "Select" },
                        { icon: "T",      tool: "top",      tip: "Top View (key 1)" },
                        { icon: "SE",     tool: "side",     tip: "Side Elevation (key 2)" },
                        { icon: "3D",     tool: "persp",    tip: "Perspective (key 3)" }
                    ]
                    Rectangle {
                        width: 40; height: 40; radius: 4
                        color: activeTool === modelData.tool ? root.colorAccent : "#2A2A2A"
                        border.color: activeTool === modelData.tool ? root.colorAccent : "transparent"
                        border.width: 2
                        Text { anchors.centerIn: parent; text: modelData.icon; color: "white"
                               font.pixelSize: modelData.icon.length > 1 ? 10 : 18; font.bold: modelData.icon.length > 1 }
                        MouseArea {
                            anchors.fill: parent; hoverEnabled: true; cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                var t = modelData.tool, HP = Math.PI/2;
                                if (t === "top")  { root.camPitch = 0;         root.camYaw = 0;   root._paintDirty = true; return; }
                                if (t === "side") { root.camPitch = HP-0.01;   root.camYaw = HP;  root._paintDirty = true; return; }
                                if (t === "persp"){ root.camPitch = -0.65;     root.camYaw = 0.7; root._paintDirty = true; return; }
                                activeTool = t;
                                if (t === "box" && boxState === "idle") createExplorationBox();
                            }
                            ToolTip.visible: containsMouse; ToolTip.text: modelData.tip
                        }
                    }
                }
            }
        }

        // ── Right sidebar (actions) ────────────────────────────────────────────
        Rectangle {
            id: rightSidebar
            anchors { right: parent.right; top: parent.top; bottom: statusBar.top }
            width: 48; color: "#1A1A1A"; opacity: 0.92; z: 10
            Column {
                anchors.centerIn: parent; spacing: 8
                Rectangle {
                    width: 40; height: 40; radius: 4; color: "#32CD32"
                    visible: boxState === "editing" || boxState === "hovering" || boxState === "resizing" || boxState === "dragging"
                    Text { anchors.centerIn: parent; text: "\u2713"; color: "white"; font.pixelSize: 20; font.bold: true }
                    MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: confirmBox()
                                ToolTip.visible: containsMouse; ToolTip.text: "Confirm Box" }
                }
                Rectangle {
                    width: 40; height: 40; radius: 4; color: "#FF4500"
                    visible: boxState !== "idle"
                    Text { anchors.centerIn: parent; text: "\u2717"; color: "white"; font.pixelSize: 20; font.bold: true }
                    MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: deleteBox()
                                ToolTip.visible: containsMouse; ToolTip.text: "Delete Box" }
                }
                Rectangle {
                    width: 40; height: 40; radius: 4; color: "#444444"; visible: boxState === "confirmed"
                    Text { anchors.centerIn: parent; text: "\uD83D\uDD12"; color: "white"; font.pixelSize: 16 }
                    MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                onClicked: { boxState = "editing"; root._paintDirty = true; }
                                ToolTip.visible: containsMouse; ToolTip.text: "Edit Box" }
                }
            }
        }

        // ── Top bar ────────────────────────────────────────────────────────────
        Rectangle {
            id: topBar
            anchors { left: leftSidebar.right; right: rightSidebar.left; top: parent.top }
            height: 36; color: "#1A1A1A"; opacity: 0.92; z: 10

            RowLayout {
                anchors.fill: parent; anchors.margins: 4; spacing: 6

                Text { text: root.title; color: "white"; font.bold: true; font.pixelSize: 12; Layout.fillWidth: true }

                // Status dot
                Text {
                    text: socket.status === WebSocket.Open ? "\u25CF Connected" : "\u25CB Disconnected"
                    color: socket.status === WebSocket.Open ? "#32CD32" : "#888"; font.pixelSize: 11
                }

                // IP input
                Rectangle {
                    width: 110; height: 26; radius: 3
                    color: "#2A2A2A"; border.color: ipInput.activeFocus ? root.colorAccent : "#555"; border.width: 1
                    TextInput {
                        id: ipInput
                        anchors.fill: parent; anchors.margins: 4
                        text: root.companionIp; color: "white"; font.pixelSize: 11; font.family: "Courier"
                        selectByMouse: true; clip: true; verticalAlignment: TextInput.AlignVCenter
                        onEditingFinished: root.companionIp = text
                        validator: RegularExpressionValidator { regularExpression: /[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/ }
                    }
                }
                Text { text: ":"; color: "#888"; font.pixelSize: 12 }

                // Port input
                Rectangle {
                    width: 48; height: 26; radius: 3
                    color: "#2A2A2A"; border.color: portInput.activeFocus ? root.colorAccent : "#555"; border.width: 1
                    TextInput {
                        id: portInput
                        anchors.fill: parent; anchors.margins: 4
                        text: root.wsPort.toString(); color: "white"; font.pixelSize: 11; font.family: "Courier"
                        selectByMouse: true; clip: true; verticalAlignment: TextInput.AlignVCenter
                        onEditingFinished: { var p = parseInt(text); if (p > 0 && p < 65536) root.wsPort = p; }
                        validator: IntValidator { bottom: 1; top: 65535 }
                    }
                }

                // Connect / Stop
                Rectangle {
                    width: 70; height: 26; radius: 3
                    color: socket.active ? "#FF4500" : root.colorAccent
                    Text { anchors.centerIn: parent; text: socket.active ? "Stop" : "Connect"; color: "white"; font.pixelSize: 11 }
                    MouseArea {
                        anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                        onClicked: {
                            if (socket.active) { socket.active = false; return; }
                            root.companionIp = ipInput.text;
                            var p = parseInt(portInput.text);
                            if (p > 0 && p < 65536) root.wsPort = p;
                            socket.active = true;
                        }
                    }
                }

                // Map / Live toggle
                Rectangle {
                    width: 60; height: 26; radius: 3
                    color: root.mapMode ? "#1A6A3A" : "#3A2A1A"
                    border.color: root.mapMode ? "#32CD32" : "#FF8C00"; border.width: 1
                    ToolTip.visible: mapModeArea.containsMouse
                    ToolTip.text: root.mapMode ? "Map mode: accumulate points (click → Live)" : "Live mode: replace each frame (click → Map)"
                    Row { anchors.centerIn: parent; spacing: 3
                          Text { text: root.mapMode ? "\u25A3" : "\u25B6"; color: root.mapMode ? "#32CD32" : "#FF8C00"; font.pixelSize: 11 }
                          Text { text: root.mapMode ? "Map" : "Live"; color: "white"; font.pixelSize: 11 }
                    }
                    MouseArea { id: mapModeArea; anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                hoverEnabled: true; onClicked: root.mapMode = !root.mapMode }
                }

                // Clear map
                Rectangle {
                    width: 50; height: 26; radius: 3; color: "#3A1A1A"
                    border.color: root.mapMode ? "#FF4444" : "#444"; border.width: 1; opacity: root.mapMode ? 1 : 0.4
                    ToolTip.visible: clearArea.containsMouse; ToolTip.text: "Clear map (" + root.totalMapPoints + " pts)"
                    Row { anchors.centerIn: parent; spacing: 3
                          Text { text: "\u2715"; color: "#FF6666"; font.pixelSize: 11 }
                          Text { text: "Clear"; color: "white"; font.pixelSize: 11 }
                    }
                    MouseArea { id: clearArea; anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                hoverEnabled: true; onClicked: if (root.mapMode) root.clearMap() }
                }

                // Point count badge
                Text {
                    visible: root.mapMode && root.totalMapPoints > 0
                    text: {
                        var drawn = root._renderCount;
                        return (root.totalMapPoints/1000).toFixed(1) + "k \u2192 " + (drawn/1000).toFixed(1) + "k drawn";
                    }
                    color: "#AAAAAA"; font.pixelSize: 10
                }
            }
        }

        // ── Status bar (telemetry) ─────────────────────────────────────────────
        Rectangle {
            id: statusBar
            anchors { left: parent.left; right: parent.right; bottom: parent.bottom }
            height: 44; color: "#1A1A1A"; opacity: 0.92; z: 10
            Row {
                anchors.centerIn: parent; spacing: 40
                Repeater {
                    model: [
                        { label: "Heading", value: root.telHeading.toFixed(1) + " °" },
                        { label: "Altitude", value: root.telAltitude.toFixed(2) + " m" },
                        { label: "Map Pts",  value: root.totalMapPoints.toString() },
                        { label: "Frames",   value: root._frameCount.toString() },
                        { label: "Drone",    value: root.dronePos ? root.dronePos.x.toFixed(1)+","+root.dronePos.y.toFixed(1)+","+root.dronePos.z.toFixed(1) : "---" }
                    ]
                    Column {
                        anchors.verticalCenter: parent.verticalCenter; spacing: 2
                        Text { text: modelData.label; color: "#CCCCCC"; font.pixelSize: 10
                               anchors.horizontalCenter: parent.horizontalCenter; font.capitalization: Font.AllUppercase }
                        Text { text: modelData.value; color: "white"; font.pixelSize: 15; font.bold: true
                               font.family: "Courier"; anchors.horizontalCenter: parent.horizontalCenter }
                    }
                }
            }
        }

        // ── Box state pill ─────────────────────────────────────────────────────
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: topBar.bottom; anchors.topMargin: 6
            height: 24; radius: 12; z: 11
            width: pillText.implicitWidth + 20
            color: boxState === "confirmed" ? "#32CD3280" : (boxState === "invalid" ? "#FF450080" : "#B8B80080")
            visible: boxState !== "idle"
            Text {
                id: pillText; anchors.centerIn: parent; color: "white"; font.pixelSize: 11
                text: {
                    switch (boxState) {
                    case "editing":   return "Box — drag to move, corners to resize";
                    case "hovering":  return "Box — hover";
                    case "dragging":  return "Moving box…";
                    case "resizing":  return "Resizing box…";
                    case "confirmed": return "Box confirmed \u2714";
                    case "invalid":   return "\u26A0 Invalid box";
                    default:          return "";
                    }
                }
            }
        }

        // ── Canvas (main viewport) ─────────────────────────────────────────────
        Canvas {
            id: canvas
            anchors { left: leftSidebar.right; right: rightSidebar.left; top: topBar.bottom; bottom: statusBar.top }
            renderStrategy: Canvas.Threaded

            property int _paintSeq: 0

            onPaint: {
                var ctx = getContext("2d");
                var W = width, H = height;
                ctx.fillStyle = "#000000";
                ctx.fillRect(0, 0, W, H);

                var flat = root._renderFlat;
                var nPts = root._renderCount;
                _paintSeq++;

                if (!flat || nPts === 0) {
                    ctx.fillStyle = "#333"; ctx.font = "14px sans-serif";
                    ctx.fillText("No points \u2014 connect to stream_server.py", W/2 - 160, H/2);
                    return;
                }

                var cxC = W/2, cyC = H/2;
                var extent = Math.max(root.cloudExtent, 1);
                var scale  = Math.min(W, H) / (extent * 1.2) * (root.camZoom / 30);
                var cosY = Math.cos(root.camYaw),   sinY = Math.sin(root.camYaw);
                var cosP = Math.cos(root.camPitch), sinP = Math.sin(root.camPitch);
                var cx_cloud = root.cloudCenterX, cy_cloud = root.cloudCenterY, cz_cloud = root.cloudCenterZ;
                var negatZ = root.negateZ;

                // Grid first (underneath points)
                ctx.strokeStyle = root.colorGrid; ctx.lineWidth = 0.5;
                ctx.font = "10px sans-serif"; ctx.fillStyle = "rgba(150,150,150,0.6)";
                var gridStep = 10;
                var gridRange = Math.max(20, Math.ceil(extent * 0.8 / gridStep) * gridStep);
                for (var gx = -gridRange; gx <= gridRange; gx += gridStep) {
                    var g0 = root.project3D(gx+cx_cloud, -gridRange+cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                    var g1 = root.project3D(gx+cx_cloud,  gridRange+cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                    ctx.beginPath(); ctx.moveTo(g0.sx, g0.sy); ctx.lineTo(g1.sx, g1.sy); ctx.stroke();
                    if (gx % 20 === 0) ctx.fillText(gx+"m", g0.sx+2, g0.sy-2);
                }
                for (var gy = -gridRange; gy <= gridRange; gy += gridStep) {
                    var h0 = root.project3D(-gridRange+cx_cloud, gy+cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                    var h1 = root.project3D( gridRange+cx_cloud, gy+cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                    ctx.beginPath(); ctx.moveTo(h0.sx, h0.sy); ctx.lineTo(h1.sx, h1.sy); ctx.stroke();
                    if (gy % 20 === 0) ctx.fillText(gy+"m", h1.sx+2, h1.sy-2);
                }

                // ── Point cloud rendering ──
                var interacting = root.isInteracting;
                var drawn = 0;

                if (interacting) {
                    // FAST PATH: single colour, 2k points, no binning
                    var fastCap = 2000;
                    var fastStep = Math.max(1, Math.ceil(nPts / fastCap)) * 6;
                    ctx.fillStyle = "#88CCCC";
                    ctx.beginPath();
                    for (var i = 0, len = nPts * 6; i < len; i += fastStep) {
                        var px = flat[i]-cx_cloud, py = flat[i+1]-cy_cloud, pz = flat[i+2]-cz_cloud;
                        var x1 = px*cosY-py*sinY, y1 = px*sinY+py*cosY;
                        var y2 = y1*cosP-(negatZ?-pz:pz)*sinP;
                        ctx.rect((cxC+x1*scale)|0, (cyC-y2*scale)|0, 2, 2);
                        drawn++;
                    }
                    ctx.fill();
                } else {
                    // QUALITY PATH: colour-binned, full render cap
                    var drawCap  = root.renderPointCap;
                    var skipStep = Math.max(1, Math.ceil(nPts / drawCap)) * 6;
                    var dotHalf  = nPts < 200 ? 3.0 : 1.5;
                    var dotSize  = dotHalf * 2;

                    var RL = 4, RGB_BINS = 64;
                    var rgbBins = new Array(RGB_BINS);
                    for (var rb = 0; rb < RGB_BINS; rb++) rgbBins[rb] = [];

                    for (var i = 0, len = nPts * 6; i < len; i += skipStep) {
                        var px = flat[i]-cx_cloud, py = flat[i+1]-cy_cloud, pz = flat[i+2]-cz_cloud;
                        var z1 = negatZ ? -pz : pz;
                        var x1 = px*cosY-py*sinY, y1 = px*sinY+py*cosY;
                        var y2 = y1*cosP-z1*sinP, z2 = y1*sinP+z1*cosP;
                        if (z2 < -extent) continue;
                        var sx = cxC+x1*scale, sy = cyC-y2*scale;
                        var binIdx = (Math.min(3,(flat[i+3]*4/256)|0))*16
                                   + (Math.min(3,(flat[i+4]*4/256)|0))*4
                                   + (Math.min(3,(flat[i+5]*4/256)|0));
                        rgbBins[binIdx].push(sx, sy);
                        drawn++;
                    }

                    for (var rb = 0; rb < RGB_BINS; rb++) {
                        var bp = rgbBins[rb]; if (!bp.length) continue;
                        var rr = ((rb>>4)&3)*85, gg = ((rb>>2)&3)*85, bb = (rb&3)*85;
                        ctx.fillStyle = "rgb("+rr+","+gg+","+bb+")";
                        ctx.beginPath();
                        for (var j = 0; j < bp.length; j += 2)
                            ctx.rect(bp[j]-dotHalf, bp[j+1]-dotHalf, dotSize, dotSize);
                        ctx.fill();
                    }
                }

                if (_paintSeq <= 3 || _paintSeq % 120 === 0)
                    console.log("[Paint] #" + _paintSeq + " drawn=" + drawn + "/" + nPts
                                + " map=" + root._mapCount);

                // Exploration box
                if (root.explorationBox && root.boxState !== "idle") {
                    var eb = root.explorationBox;
                    var bMinX=eb.cx-eb.dx, bMaxX=eb.cx+eb.dx;
                    var bMinY=eb.cy-eb.dy, bMaxY=eb.cy+eb.dy;
                    var bMinZ=eb.cz-eb.dz, bMaxZ=eb.cz+eb.dz;
                    var boxClr = String(root._boxColor());
                    var rH = parseInt(boxClr.substring(1,3),16)||180;
                    var gH = parseInt(boxClr.substring(3,5),16)||180;
                    var bH = parseInt(boxClr.substring(5,7),16)||0;
                    var faces = [
                        [{x:bMinX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMinZ},{x:bMinX,y:bMaxY,z:bMinZ}],
                        [{x:bMinX,y:bMinY,z:bMaxZ},{x:bMaxX,y:bMinY,z:bMaxZ},{x:bMaxX,y:bMaxY,z:bMaxZ},{x:bMinX,y:bMaxY,z:bMaxZ}],
                        [{x:bMinX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMinY,z:bMaxZ},{x:bMinX,y:bMinY,z:bMaxZ}],
                        [{x:bMinX,y:bMaxY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMaxZ},{x:bMinX,y:bMaxY,z:bMaxZ}],
                        [{x:bMinX,y:bMinY,z:bMinZ},{x:bMinX,y:bMaxY,z:bMinZ},{x:bMinX,y:bMaxY,z:bMaxZ},{x:bMinX,y:bMinY,z:bMaxZ}],
                        [{x:bMaxX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMaxZ},{x:bMaxX,y:bMinY,z:bMaxZ}]
                    ];
                    var op = root._boxOpacity();
                    for (var fi = 0; fi < faces.length; fi++) {
                        var fc = faces[fi], fp = [];
                        for (var fj = 0; fj < fc.length; fj++)
                            fp.push(root.project3D(fc[fj].x,fc[fj].y,fc[fj].z,cxC,cyC,scale,cosY,sinY,cosP,sinP));
                        ctx.fillStyle = "rgba("+rH+","+gH+","+bH+","+(op*0.6).toFixed(2)+")";
                        ctx.beginPath(); ctx.moveTo(fp[0].sx, fp[0].sy);
                        for (var fk = 1; fk < fp.length; fk++) ctx.lineTo(fp[fk].sx, fp[fk].sy);
                        ctx.closePath(); ctx.fill();
                    }
                    // Wireframe
                    var wfCorners = [{x:bMinX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMinY,z:bMinZ},{x:bMaxX,y:bMaxY,z:bMinZ},{x:bMinX,y:bMaxY,z:bMinZ},
                                     {x:bMinX,y:bMinY,z:bMaxZ},{x:bMaxX,y:bMinY,z:bMaxZ},{x:bMaxX,y:bMaxY,z:bMaxZ},{x:bMinX,y:bMaxY,z:bMaxZ}];
                    var sc = wfCorners.map(function(c){ return root.project3D(c.x,c.y,c.z,cxC,cyC,scale,cosY,sinY,cosP,sinP); });
                    ctx.strokeStyle = boxClr; ctx.lineWidth = 2; ctx.beginPath();
                    ctx.moveTo(sc[0].sx,sc[0].sy); ctx.lineTo(sc[1].sx,sc[1].sy); ctx.lineTo(sc[2].sx,sc[2].sy);
                    ctx.lineTo(sc[3].sx,sc[3].sy); ctx.lineTo(sc[0].sx,sc[0].sy);
                    ctx.moveTo(sc[4].sx,sc[4].sy); ctx.lineTo(sc[5].sx,sc[5].sy); ctx.lineTo(sc[6].sx,sc[6].sy);
                    ctx.lineTo(sc[7].sx,sc[7].sy); ctx.lineTo(sc[4].sx,sc[4].sy);
                    ctx.moveTo(sc[0].sx,sc[0].sy); ctx.lineTo(sc[4].sx,sc[4].sy);
                    ctx.moveTo(sc[1].sx,sc[1].sy); ctx.lineTo(sc[5].sx,sc[5].sy);
                    ctx.moveTo(sc[2].sx,sc[2].sy); ctx.lineTo(sc[6].sx,sc[6].sy);
                    ctx.moveTo(sc[3].sx,sc[3].sy); ctx.lineTo(sc[7].sx,sc[7].sy); ctx.stroke();
                    // Handles
                    if (boxState === "editing"||boxState === "hovering"||boxState === "resizing"||boxState === "dragging") {
                        for (var hi = 0; hi < 8; hi++) {
                            var hp = sc[hi], hs = (hi === activeHandleIdx && boxState === "resizing") ? 9 : 6;
                            ctx.fillStyle = hi === activeHandleIdx ? "#FFF" : boxClr;
                            ctx.strokeStyle = "#FFF"; ctx.lineWidth = 1.5;
                            ctx.beginPath(); ctx.arc(hp.sx, hp.sy, hs, 0, 2*Math.PI); ctx.fill(); ctx.stroke();
                        }
                    }
                }

                // Waypoints
                for (var wi = 0; wi < root.waypoints.length; wi++) {
                    var wp = root.waypoints[wi];
                    var wpp = root.project3D(wp.x,wp.y,wp.z,cxC,cyC,scale,cosY,sinY,cosP,sinP);
                    var isSel = wi === root.selectedWaypointIdx;
                    var wpR = isSel ? 10 : 8;
                    if (wi > 0) {
                        var pw = root.waypoints[wi-1];
                        var pwp = root.project3D(pw.x,pw.y,pw.z,cxC,cyC,scale,cosY,sinY,cosP,sinP);
                        ctx.strokeStyle = "rgba(255,200,0,0.6)"; ctx.lineWidth = 2; ctx.setLineDash([]);
                        ctx.beginPath(); ctx.moveTo(pwp.sx,pwp.sy); ctx.lineTo(wpp.sx,wpp.sy); ctx.stroke();
                    }
                    ctx.fillStyle = wi === root.draggingWaypointIdx ? "#FFF" : (isSel ? "#FFA500" : root.colorWaypoint);
                    ctx.beginPath(); ctx.arc(wpp.sx, wpp.sy, wpR, 0, 2*Math.PI); ctx.fill();
                    ctx.strokeStyle = isSel ? root.colorAccent : "#FFF"; ctx.lineWidth = isSel ? 2.5 : 1.5;
                    ctx.beginPath(); ctx.arc(wpp.sx, wpp.sy, wpR, 0, 2*Math.PI); ctx.stroke();
                    ctx.fillStyle = "#000"; ctx.font = "bold 10px sans-serif";
                    var ns = String(wp.id); ctx.fillText(ns, wpp.sx-(ns.length>1?6:3), wpp.sy+4);
                    ctx.fillStyle = isSel ? "rgba(0,206,209,0.95)" : "rgba(255,255,200,0.75)";
                    ctx.font = "10px sans-serif";
                    ctx.fillText("["+wp.x.toFixed(1)+","+wp.y.toFixed(1)+","+wp.z.toFixed(1)+"]", wpp.sx+wpR+4, wpp.sy-4);
                }

                // Drone marker
                if (root.dronePos) {
                    var dp = root.dronePos;
                    var dpP = root.project3D(dp.x,dp.y,dp.z,cxC,cyC,scale,cosY,sinY,cosP,sinP);
                    var dsx=dpP.sx, dsy=dpP.sy, ds=12;
                    ctx.strokeStyle = "rgba(65,105,225,0.4)"; ctx.lineWidth = 3;
                    ctx.beginPath(); ctx.arc(dsx,dsy,ds+8,0,2*Math.PI); ctx.stroke();
                    ctx.fillStyle = "rgba(65,105,225,0.8)";
                    ctx.beginPath(); ctx.arc(dsx,dsy,ds,0,2*Math.PI); ctx.fill();
                    ctx.strokeStyle = "#4169E1"; ctx.lineWidth = 2;
                    ctx.beginPath(); ctx.arc(dsx,dsy,ds,0,2*Math.PI); ctx.stroke();
                    ctx.strokeStyle = "#FFF"; ctx.lineWidth = 1;
                    ctx.beginPath();
                    ctx.moveTo(dsx-ds*0.6,dsy); ctx.lineTo(dsx+ds*0.6,dsy);
                    ctx.moveTo(dsx,dsy-ds*0.6); ctx.lineTo(dsx,dsy+ds*0.6); ctx.stroke();
                    var yawR = dp.yaw * Math.PI / 180;
                    var hx=dsx+Math.sin(yawR)*(ds+10), hy=dsy-Math.cos(yawR)*(ds+10);
                    ctx.strokeStyle = "#FF4444"; ctx.lineWidth = 2;
                    ctx.beginPath(); ctx.moveTo(dsx,dsy); ctx.lineTo(hx,hy); ctx.stroke();
                    ctx.fillStyle = "#4169E1"; ctx.font = "bold 10px sans-serif";
                    ctx.fillText("["+dp.x.toFixed(0)+","+dp.y.toFixed(0)+"]", dsx+ds+4, dsy-ds);
                }

                // Selected point
                if (root.selectedPoint) {
                    var sp = root.selectedPoint;
                    var spp = root.project3D(sp.x,sp.y,sp.z,cxC,cyC,scale,cosY,sinY,cosP,sinP);
                    ctx.strokeStyle = "red"; ctx.lineWidth = 2;
                    ctx.beginPath(); ctx.arc(spp.sx,spp.sy,8,0,2*Math.PI); ctx.stroke();
                    ctx.fillStyle = "white"; ctx.font = "12px sans-serif";
                    ctx.fillText("TARGET ("+root.distToTarget.toFixed(2)+"m)", spp.sx+12, spp.sy);
                }

                // Axis gizmo
                var gox = width-60, goy = height-55, axLen = 30;
                var oScr  = root.project3D(cx_cloud, cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                var xTipS = root.project3D(cx_cloud+extent*0.5, cy_cloud, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                var yTipS = root.project3D(cx_cloud, cy_cloud+extent*0.5, cz_cloud, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                var zTipS = root.project3D(cx_cloud, cy_cloud, cz_cloud+extent*0.5, cxC, cyC, scale, cosY, sinY, cosP, sinP);
                function gNorm(tip, o, l) {
                    var dx=tip.sx-o.sx, dy=tip.sy-o.sy, m=Math.sqrt(dx*dx+dy*dy);
                    return m < 0.001 ? {dx:0,dy:0} : {dx:dx/m*l,dy:dy/m*l};
                }
                var xD=gNorm(xTipS,oScr,axLen), yD=gNorm(yTipS,oScr,axLen), zD=gNorm(zTipS,oScr,axLen);
                ctx.fillStyle = "rgba(0,0,0,0.55)";
                ctx.beginPath(); ctx.arc(gox,goy,42,0,2*Math.PI); ctx.fill();
                function gAxis(d, clr, lbl) {
                    if (Math.abs(d.dx)<0.5 && Math.abs(d.dy)<0.5) return;
                    ctx.strokeStyle = clr; ctx.lineWidth = 2.5;
                    ctx.beginPath(); ctx.moveTo(gox,goy); ctx.lineTo(gox+d.dx,goy+d.dy); ctx.stroke();
                    var ang=Math.atan2(d.dy,d.dx);
                    ctx.fillStyle = clr; ctx.beginPath();
                    ctx.moveTo(gox+d.dx,goy+d.dy);
                    ctx.lineTo(gox+d.dx-7*Math.cos(ang-0.4),goy+d.dy-7*Math.sin(ang-0.4));
                    ctx.lineTo(gox+d.dx-7*Math.cos(ang+0.4),goy+d.dy-7*Math.sin(ang+0.4));
                    ctx.closePath(); ctx.fill();
                    ctx.font="bold 11px sans-serif"; ctx.fillText(lbl,gox+d.dx*1.35-4,goy+d.dy*1.35+4);
                }
                gAxis(xD,"#FF6666","X"); gAxis(yD,"#66DD66","Y"); gAxis(zD,"#6699FF","Z");
            }
        }

        // ── Zoom overlay (bottom-left) ────────────────────────────────────────
        Rectangle {
            anchors { left: leftSidebar.right; bottom: statusBar.top; margins: 8 }
            width: zoomCol.width+12; height: zoomCol.height+12
            color: "#191919"; opacity: 0.85; radius: 4
            border.color: root.colorAccent; border.width: 1; z: 10
            Column { id: zoomCol; x: 6; y: 6; spacing: 4
                Text { text: "Zoom: "+root.camZoom.toFixed(1) + (root.viewLocked ? " \uD83D\uDD12" : " auto"); color: "white"; font.pixelSize: 11 }
                Text { visible: !!root.selectedPoint
                       text: root.selectedPoint ? "Target: "+root.selectedPoint.x.toFixed(1)+", "
                             +root.selectedPoint.y.toFixed(1)+", "+root.selectedPoint.z.toFixed(1)
                             +" (D: "+root.distToTarget.toFixed(1)+"m)" : ""
                       color: root.colorAccent; font.pixelSize: 10; font.bold: true }
                Row { spacing: 4
                    Rectangle { width:28;height:24;radius:3;color:"#2A2A2A"
                        Text{anchors.centerIn:parent;text:"+";color:"white";font.pixelSize:14}
                        MouseArea{anchors.fill:parent;cursorShape:Qt.PointingHandCursor
                            onClicked:{root._startInteraction();root.camZoom=Math.min(500,root.camZoom*1.3);root._paintDirty=true;}} }
                    Rectangle { width:28;height:24;radius:3;color:"#2A2A2A"
                        Text{anchors.centerIn:parent;text:"\u2212";color:"white";font.pixelSize:14}
                        MouseArea{anchors.fill:parent;cursorShape:Qt.PointingHandCursor
                            onClicked:{root._startInteraction();root.camZoom=Math.max(2,root.camZoom/1.3);root._paintDirty=true;}} }
                }
                Row { spacing: 4
                    Rectangle { width:60;height:24;radius:3;color: root.dronePos ? "#1A3A6A" : "#2A2A2A"
                        Text{anchors.centerIn:parent;text:"\u2316 Drone";color: root.dronePos ? "#6699FF" : "#666";font.pixelSize:10}
                        MouseArea{anchors.fill:parent;cursorShape:Qt.PointingHandCursor
                            onClicked: root.centerOnDrone()
                            ToolTip.visible:containsMouse; ToolTip.text:"Center view on drone (C)"} }
                    Rectangle { width:52;height:24;radius:3;color:"#2A2A2A"
                        Text{anchors.centerIn:parent;text:"\u25A3 Fit";color:"#AAA";font.pixelSize:10}
                        MouseArea{anchors.fill:parent;cursorShape:Qt.PointingHandCursor
                            onClicked: root.fitAll()
                            ToolTip.visible:containsMouse; ToolTip.text:"Fit all points in view (F)"} }
                }
            }
        }

        // ── Mouse interaction ─────────────────────────────────────────────────
        MouseArea {
            anchors.fill: canvas
            hoverEnabled: true
            acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
            z: canvas.z + 1

            property point lastPos
            property bool  isDrag: false
            property string mode: "none"

            onPressed: function(mouse) {
                lastPos = Qt.point(mouse.x, mouse.y); isDrag = false; mode = "none";
                root.focus = true;
                if (mouse.button === Qt.MiddleButton ||
                    (mouse.button === Qt.LeftButton && (mouse.modifiers & Qt.AltModifier))) {
                    mode = "pan"; return;
                }
                if (mouse.button !== Qt.LeftButton) { mode = "orbit"; return; }
                if (root.activeTool === "waypoint" || root.activeTool === "select") {
                    var wpH = root.hitTestWaypoint(mouse.x, mouse.y);
                    if (wpH >= 0) {
                        mode = "wpDrag"; root.draggingWaypointIdx = wpH; root.selectedWaypointIdx = wpH;
                        var ww = root.waypoints[wpH];
                        _wpDragStart = { mouseX: mouse.x, mouseY: mouse.y, wx: ww.x, wy: ww.y, wz: ww.z };
                        root._paintDirty = true; return;
                    }
                }
                if (root.activeTool === "box" && root.explorationBox && root.boxState !== "confirmed") {
                    var hr = hitTestBox(mouse.x, mouse.y);
                    if (hr.type === "handle") {
                        mode = "boxResize"; root.activeHandleIdx = hr.index; root.boxState = "resizing";
                        root._dragStartMouse = Qt.point(mouse.x, mouse.y);
                        root._dragStartBox = { cx:root.explorationBox.cx, cy:root.explorationBox.cy, cz:root.explorationBox.cz,
                                               dx:root.explorationBox.dx, dy:root.explorationBox.dy, dz:root.explorationBox.dz };
                        root._paintDirty = true; return;
                    } else if (hr.type === "face") {
                        mode = "boxDrag"; root.boxState = "dragging";
                        root._dragStartMouse = Qt.point(mouse.x, mouse.y);
                        root._dragStartBox = { cx:root.explorationBox.cx, cy:root.explorationBox.cy, cz:root.explorationBox.cz,
                                               dx:root.explorationBox.dx, dy:root.explorationBox.dy, dz:root.explorationBox.dz };
                        root._paintDirty = true; return;
                    }
                }
                mode = "orbit";
            }

            onPositionChanged: function(mouse) {
                var dx = mouse.x - lastPos.x, dy = mouse.y - lastPos.y;
                if (Math.abs(dx)>2||Math.abs(dy)>2) isDrag = true;
                if (mode === "orbit") {
                    root._startInteraction();
                    root.camYaw += dx*0.01;
                    root.camPitch = Math.max(-(Math.PI/2-0.02), Math.min(Math.PI/2-0.02, root.camPitch+dy*0.01));
                    lastPos = Qt.point(mouse.x, mouse.y); root._paintDirty = true;
                } else if (mode === "pan") {
                    root._startInteraction();
                    var ps = root.cloudExtent*0.002*(30/root.camZoom);
                    root.cloudCenterX -= dx*ps*Math.cos(root.camYaw);
                    root.cloudCenterY -= dx*ps*Math.sin(root.camYaw);
                    root.cloudCenterZ += dy*ps*0.5;
                    lastPos = Qt.point(mouse.x, mouse.y); root._paintDirty = true;
                } else if (mode === "wpDrag" && root.draggingWaypointIdx >= 0 && root._wpDragStart) {
                    var ww = root.waypoints[root.draggingWaypointIdx];
                    var np = root.unproject2D(mouse.x, mouse.y, ww.z);
                    root.updateWaypoint(root.draggingWaypointIdx, np.x, np.y, ww.z);
                } else if (mode === "boxDrag" && root.explorationBox && root._dragStartBox) {
                    var wpp2 = root.cloudExtent/Math.min(canvas.width,canvas.height)*1.2*(30/root.camZoom);
                    var tx=(mouse.x-root._dragStartMouse.x)*wpp2, ty=-(mouse.y-root._dragStartMouse.y)*wpp2;
                    var cy2=Math.cos(root.camYaw), sy2=Math.sin(root.camYaw);
                    root.explorationBox = {
                        cx:root._dragStartBox.cx+tx*cy2+ty*sy2*Math.cos(root.camPitch),
                        cy:root._dragStartBox.cy+tx*sy2-ty*cy2*Math.cos(root.camPitch),
                        cz:root._dragStartBox.cz+ty*Math.sin(root.camPitch),
                        dx:root._dragStartBox.dx, dy:root._dragStartBox.dy, dz:root._dragStartBox.dz };
                    root.boxState = root.validateBox() ? "dragging" : "invalid"; root._paintDirty = true;
                } else if (mode === "boxResize" && root.explorationBox && root._dragStartBox && root.activeHandleIdx >= 0) {
                    var wpp3 = root.cloudExtent/Math.min(canvas.width,canvas.height)*1.2*(30/root.camZoom);
                    var rdx=(mouse.x-root._dragStartMouse.x)*wpp3, rdy=-(mouse.y-root._dragStartMouse.y)*wpp3;
                    var sx=(root.activeHandleIdx%4===1||root.activeHandleIdx%4===2)?1:-1;
                    var sy=(root.activeHandleIdx%4>=2)?1:-1, sz=(root.activeHandleIdx>=4)?1:-1;
                    var ndx=Math.max(root.boxMinSize/2,root._dragStartBox.dx+sx*rdx*0.5);
                    var ndy=Math.max(root.boxMinSize/2,root._dragStartBox.dy+sy*rdx*0.3);
                    var ndz=Math.max(root.boxMinSize*0.3,root._dragStartBox.dz+sz*rdy*0.5);
                    root.explorationBox = {
                        cx:root._dragStartBox.cx+sx*(ndx-root._dragStartBox.dx)*0.5,
                        cy:root._dragStartBox.cy+sy*(ndy-root._dragStartBox.dy)*0.5,
                        cz:root._dragStartBox.cz+sz*(ndz-root._dragStartBox.dz)*0.5,
                        dx:ndx, dy:ndy, dz:ndz };
                    root.boxState = root.validateBox() ? "resizing" : "invalid"; root._paintDirty = true;
                }
            }

            onReleased: function(mouse) {
                if (mode === "boxDrag") root.boxState = root.validateBox() ? "editing" : "invalid";
                else if (mode === "boxResize") { root.activeHandleIdx = -1; root.boxState = root.validateBox() ? "editing" : "invalid"; }
                else if (mode === "wpDrag")  { root.draggingWaypointIdx = -1; root._wpDragStart = null; }
                mode = "none"; root._paintDirty = true;
            }

            onClicked: function(mouse) {
                if (isDrag) return;
                if (root.activeTool === "waypoint") {
                    var wh = root.hitTestWaypoint(mouse.x, mouse.y);
                    if (wh >= 0) { root.selectedWaypointIdx = wh; root._paintDirty = true; return; }
                    var gz = root.cloudBounds ? root.cloudBounds.minZ : root.cloudCenterZ;
                    var wp2 = root.unproject2D(mouse.x, mouse.y, gz+2.4);
                    root.addWaypoint(wp2.x, wp2.y, wp2.z); return;
                }
                if (root.activeTool === "select") {
                    var sh = root.hitTestWaypoint(mouse.x, mouse.y);
                    root.selectedWaypointIdx = (sh >= 0 && sh !== root.selectedWaypointIdx) ? sh : -1;
                    root._paintDirty = true; return;
                }
                if (root.activeTool === "box" && root.boxState === "confirmed") {
                    root.boxState = "editing"; root._paintDirty = true; return;
                }
                // Point pick (uses flat render buffer)
                var mx=mouse.x, my=mouse.y, minD=20, bestP=null;
                var pf=root._renderFlat, pn=root._renderCount;
                if (!pf || pn===0) return;
                var extent2=Math.max(root.cloudExtent,1);
                var sc2=Math.min(canvas.width,canvas.height)/(extent2*1.2)*(root.camZoom/30);
                var cosY2=Math.cos(root.camYaw),sinY2=Math.sin(root.camYaw);
                var cosP2=Math.cos(root.camPitch),sinP2=Math.sin(root.camPitch);
                var pickStep=Math.max(6, Math.ceil(pn/3000)*6);
                for (var ip=0;ip<pn*6;ip+=pickStep) {
                    var pr=root.project3D(pf[ip],pf[ip+1],pf[ip+2],canvas.width/2,canvas.height/2,sc2,cosY2,sinY2,cosP2,sinP2);
                    var pd=Math.sqrt((pr.sx-mx)*(pr.sx-mx)+(pr.sy-my)*(pr.sy-my));
                    if (pd<minD){minD=pd;bestP={x:pf[ip],y:pf[ip+1],z:pf[ip+2]};}
                }
                root.selectedPoint = bestP;
                if (bestP) root.distToTarget=Math.sqrt(bestP.x*bestP.x+bestP.y*bestP.y+bestP.z*bestP.z);
                root._paintDirty = true;
            }

            onDoubleClicked: function(mouse) {
                var wd = root.hitTestWaypoint(mouse.x, mouse.y);
                if (wd >= 0) { root.deleteWaypoint(wd); return; }
                if (root.explorationBox && root.boxState === "confirmed") {
                    root.boxState = "editing"; root._paintDirty = true;
                }
            }

            onWheel: function(wheel) {
                root._startInteraction();
                var factor = wheel.angleDelta.y > 0 ? 1.12 : (1.0/1.12);
                root.camZoom = Math.max(2, Math.min(500, root.camZoom * factor));
                root._paintDirty = true;
            }

            function hitTestBox(mx, my) {
                if (!root.explorationBox) return { type: "none", index: -1 };
                var eb = root.explorationBox;
                var extent3=Math.max(root.cloudExtent,1);
                var sc3=Math.min(canvas.width,canvas.height)/(extent3*1.2)*(root.camZoom/30);
                var cosYh=Math.cos(root.camYaw),sinYh=Math.sin(root.camYaw);
                var cosPh=Math.cos(root.camPitch),sinPh=Math.sin(root.camPitch);
                var hc = [
                    {x:eb.cx-eb.dx,y:eb.cy-eb.dy,z:eb.cz-eb.dz},{x:eb.cx+eb.dx,y:eb.cy-eb.dy,z:eb.cz-eb.dz},
                    {x:eb.cx+eb.dx,y:eb.cy+eb.dy,z:eb.cz-eb.dz},{x:eb.cx-eb.dx,y:eb.cy+eb.dy,z:eb.cz-eb.dz},
                    {x:eb.cx-eb.dx,y:eb.cy-eb.dy,z:eb.cz+eb.dz},{x:eb.cx+eb.dx,y:eb.cy-eb.dy,z:eb.cz+eb.dz},
                    {x:eb.cx+eb.dx,y:eb.cy+eb.dy,z:eb.cz+eb.dz},{x:eb.cx-eb.dx,y:eb.cy+eb.dy,z:eb.cz+eb.dz}
                ];
                for (var hi=0;hi<8;hi++) {
                    var hp=root.project3D(hc[hi].x,hc[hi].y,hc[hi].z,canvas.width/2,canvas.height/2,sc3,cosYh,sinYh,cosPh,sinPh);
                    if (Math.sqrt((hp.sx-mx)*(hp.sx-mx)+(hp.sy-my)*(hp.sy-my))<15) return {type:"handle",index:hi};
                }
                var cp=root.project3D(eb.cx,eb.cy,eb.cz,canvas.width/2,canvas.height/2,sc3,cosYh,sinYh,cosPh,sinPh);
                if (Math.sqrt((cp.sx-mx)*(cp.sx-mx)+(cp.sy-my)*(cp.sy-my))<Math.max(eb.dx,eb.dy)*sc3*1.2) return {type:"face",index:-1};
                return {type:"none",index:-1};
            }
        }
    }

    // ── Keyboard shortcuts ────────────────────────────────────────────────────
    Keys.onPressed: function(event) {
        if (event.key===Qt.Key_Return||event.key===Qt.Key_Enter) {
            if (boxState==="editing"||boxState==="hovering") confirmBox(); event.accepted=true;
        } else if (event.key===Qt.Key_Escape) {
            if (selectedWaypointIdx>=0){selectedWaypointIdx=-1;root._paintDirty=true;}
            else if (boxState!=="idle"&&boxState!=="confirmed"){deleteBox();activeTool="orbit";}
            event.accepted=true;
        } else if (event.key===Qt.Key_Delete||event.key===Qt.Key_Backspace) {
            if (selectedWaypointIdx>=0) deleteWaypoint(selectedWaypointIdx);
            else if (boxState==="confirmed") deleteBox();
            event.accepted=true;
        } else if (event.key===Qt.Key_1){camPitch=0;camYaw=0;root._paintDirty=true;event.accepted=true;}
        else if (event.key===Qt.Key_2){camPitch=Math.PI/2-0.01;camYaw=Math.PI/2;root._paintDirty=true;event.accepted=true;}
        else if (event.key===Qt.Key_3){camPitch=-0.65;camYaw=0.7;root._paintDirty=true;event.accepted=true;}
        else if (event.key===Qt.Key_C){centerOnDrone();event.accepted=true;}
        else if (event.key===Qt.Key_F){fitAll();event.accepted=true;}
    }
    focus: true
}
