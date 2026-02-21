import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QGroundControl
import QGroundControl.Controls
import QtWebSockets
import "FlyViewPointCloudData.js" as PointCloudData

Rectangle {
    id: root

    property bool allowClose:          true
    property var  pointCloud:          PointCloudData.demoCloud()
    property string title:             qsTr("Point Cloud (Canvas 2.5D POC)")

    // Simple camera state for 2.5D projection
    property real camPitch: 0.5
    property real camYaw: 0.0
    property real camZoom: 30.0

    // Picking state
    property var selectedPoint: null
    property real distToTarget: 0.0

    // Auto-fit: center and extent of current cloud (so real-world coords scale correctly)
    property real cloudCenterX: 0
    property real cloudCenterY: 0
    property real cloudCenterZ: 0
    property real cloudExtent: 20
    property int _debugFrameCount: 0
    property bool binaryBigEndian: false
    property bool binary12Bytes: false  // false = 16 B/pt (x,y,z,intensity); true = legacy 12 B/pt (x,y,z only)
    property bool binary16Skip4: false
    property bool binaryCountHeader: false   // set true if server uses --count-header
    property string axisOrder: "xyz"        // try "yxz", "xzy", "zxy" etc. if cloud looks wrong
    property bool negateZ: true             // true = Z-up display (NED data); false = use Z as-is
    property var _lastRawPoints: []         // last received points before axis order (so we can re-apply)
    property var cloudBounds: null          // full bounds { minX, maxX, ... } for drawing box
    property var dronePos: null             // { x, y, z, yaw } from frame header
    property real _padding: 10

    Component.onCompleted: {
        PointCloudData.setBinaryBigEndian(binaryBigEndian)
        PointCloudData.setBinary12Bytes(binary12Bytes)
        PointCloudData.setBinary16Skip4(binary16Skip4)
        PointCloudData.setBinaryCountHeader(binaryCountHeader)
    }

    WebSocket {
        id: socket
        url: "ws://10.42.0.1:8765"
        active: false

        // result = { points: [], dronePos: {x,y,z,yaw}|null }
        function applyPointCloud(result, sourceLabel) {
            var pts = result.points;
            if (!pts || pts.length === 0) return;
            _lastRawPoints = pts;

            // Update drone position (apply same axis order)
            if (result.dronePos) {
                var dp = result.dronePos;
                var dArr = PointCloudData.applyAxisOrder([{x: dp.x, y: dp.y, z: dp.z, intensity: 0}], axisOrder);
                dronePos = { x: dArr[0].x, y: dArr[0].y, z: dArr[0].z, yaw: dp.yaw };
            }

            pts = PointCloudData.applyAxisOrder(pts, axisOrder);
            var bounds = PointCloudData.computeBounds(pts);
            cloudBounds = bounds;
            cloudCenterX = bounds.centerX;
            cloudCenterY = bounds.centerY;
            cloudCenterZ = bounds.centerZ;
            cloudExtent = bounds.extent;
            pointCloud = pts;
            canvas.requestPaint();
            title = "Live " + sourceLabel + ": " + pts.length + " pts";
            _debugFrameCount++;
            if (_debugFrameCount <= 3 || _debugFrameCount % 30 === 0) {
                console.log("[PointCloud] " + sourceLabel + " frame #" + _debugFrameCount + " pts=" + pts.length +
                    " center=(" + bounds.centerX.toFixed(2) + "," + bounds.centerY.toFixed(2) + "," + bounds.centerZ.toFixed(2) + ")" +
                    " extent=" + bounds.extent.toFixed(2) + " first=(" + (pts[0].x.toFixed(2)) + "," + (pts[0].y.toFixed(2)) + "," + (pts[0].z.toFixed(2)) + ")" +
                    (dronePos ? " drone=(" + dronePos.x.toFixed(2) + "," + dronePos.y.toFixed(2) + "," + dronePos.z.toFixed(2) + ")" : ""));
            }
        }

        onBinaryMessageReceived: function(message) {
            var buf = message;
            if (typeof message === 'string') {
                var arr = new ArrayBuffer(message.length);
                var view = new Uint8Array(arr);
                for (var i = 0; i < message.length; i++) view[i] = message.charCodeAt(i);
                buf = arr;
            }
            var debugFirst = _debugFrameCount === 0;
            var result = PointCloudData.parseRawBinary(buf, debugFirst);
            if (result.points.length > 0) {
                applyPointCloud(result, "Binary");
            } else {
                console.log("[PointCloud] raw binary parse returned 0 points");
            }
        }

        onTextMessageReceived: function(message) {
            var isBinary = message.indexOf(',') === -1;
            if (isBinary) {
                 var debugFirst = _debugFrameCount === 0;
                 var result = PointCloudData.parseBase64Binary(message, debugFirst);
                 if (result.points.length > 0) {
                     applyPointCloud(result, "Base64");
                 } else {
                     console.log("[PointCloud] Base64 parse returned 0 points, message length=" + message.length);
                 }
            } else {
                var pts = PointCloudData.parseCSV(message);
                if (pts.length > 0) {
                    applyPointCloud({ points: pts, dronePos: null }, "CSV");
                } else {
                    console.log("[PointCloud] csv parse returned 0 points, lines=" + message.split('\n').length);
                }
            }
        }

        onStatusChanged: {
            if (socket.status == WebSocket.Error) {
                console.log("[PointCloud] WebSocket Error: " + socket.errorString);
                statusLabel.text = "Error: " + socket.errorString
            } else if (socket.status == WebSocket.Open) {
                console.log("[PointCloud] Connected to " + socket.url);
                statusLabel.text = "Connected"
            } else if (socket.status == WebSocket.Closed) {
                console.log("[PointCloud] WebSocket closed");
                statusLabel.text = "Disconnected"
            } else if (socket.status == WebSocket.Connecting) {
                console.log("[PointCloud] Connecting to " + socket.url + " ...");
                statusLabel.text = "Connecting..."
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            QGCLabel {
                text: title
                color: "white"
                font.bold: true
                Layout.fillWidth: true
            }

            QGCLabel {
                id: statusLabel
                text: "Disconnected"
                color: socket.status === WebSocket.Open ? "green" : (socket.status === WebSocket.Error ? "red" : "white")
            }

            QGCButton {
                text: socket.active ? "Disconnect" : "Connect"
                onClicked: {
                    socket.active = !socket.active
                }
            }


            QGCButton {
                text: qsTr("Reload")
                onClicked: {
                    var pts = PointCloudData.demoCloud()
                    _lastRawPoints = pts
                    pts = PointCloudData.applyAxisOrder(pts, axisOrder)
                    var bounds = PointCloudData.computeBounds(pts)
                    cloudCenterX = bounds.centerX
                    cloudCenterY = bounds.centerY
                    cloudCenterZ = bounds.centerZ
                    cloudExtent = bounds.extent
                    pointCloud = pts
                    canvas.requestPaint()
                    title = "Point Cloud (Canvas 2.5D POC)"
                }
            }

            /*
            QGCCheckBox {
                text: qsTr("Big Endian")
                checked: binaryBigEndian
                onCheckedChanged: {
                    binaryBigEndian = checked
                    PointCloudData.setBinaryBigEndian(checked)
                }
            }
            QGCCheckBox {
                text: qsTr("12 B/pt (x,y,z)")
                checked: binary12Bytes
                onCheckedChanged: {
                    binary12Bytes = checked
                    PointCloudData.setBinary12Bytes(checked)
                }
            }
            QGCCheckBox {
                text: qsTr("16 B skip 4")
                checked: binary16Skip4
                enabled: !binary12Bytes
                onCheckedChanged: {
                    binary16Skip4 = checked
                    PointCloudData.setBinary16Skip4(checked)
                }
            }
            QGCCheckBox {
                text: qsTr("Count header")
                checked: binaryCountHeader
                enabled: !binary12Bytes
                onCheckedChanged: {
                    binaryCountHeader = checked
                    PointCloudData.setBinaryCountHeader(checked)
                }
            }
            */

            QGCLabel { text: qsTr("Axes:"); color: "white" }
            ComboBox {
                id: axisOrderCombo
                model: ["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"]
                currentIndex: Math.max(0, model.indexOf(root.axisOrder))
                onActivated: function(i) {
                    root.axisOrder = model[i]
                    if (_lastRawPoints && _lastRawPoints.length > 0) {
                        var pts = PointCloudData.applyAxisOrder(_lastRawPoints, root.axisOrder)
                        var bounds = PointCloudData.computeBounds(pts)
                        cloudBounds = bounds
                        cloudCenterX = bounds.centerX
                        cloudCenterY = bounds.centerY
                        cloudCenterZ = bounds.centerZ
                        cloudExtent = bounds.extent
                        pointCloud = pts
                        if (root.dronePos) {
                            var dp = root.dronePos;
                            var dArr = PointCloudData.applyAxisOrder([{x: dp.x, y: dp.y, z: dp.z, intensity: 0}], root.axisOrder);
                            root.dronePos = { x: dArr[0].x, y: dArr[0].y, z: dArr[0].z, yaw: dp.yaw };
                        }
                        canvas.requestPaint()
                    }
                }
            }
            QGCCheckBox {
                text: qsTr("Z up")
                checked: negateZ
                onCheckedChanged: {
                    negateZ = checked
                    canvas.requestPaint()
                }
            }

            QGCButton {
                text: qsTr("Load CSV")
                onClicked: loadFile("file:///Users/thomas/Desktop/points.csv")
            }

            QGCButton {
                visible: allowClose
                text: qsTr("X")
                onClicked: root.visible = false
            }
        }

        Item {
            id: viewHolder
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            Canvas {
                id: canvas
                anchors.fill: parent
                renderStrategy: Canvas.Threaded
                property int _paintCount: 0

                onPaint: {
                    var ctx = getContext("2d");
                    ctx.fillStyle = "#101012"; // Background
                    ctx.fillRect(0, 0, width, height);

                    var cx = width / 2;
                    var cy = height / 2;
                    var points = root.pointCloud;

                    if (!points) return;

                    var extent = Math.max(root.cloudExtent, 1);
                    var scaleToFit = Math.min(width, height) / (extent * 1.2) * (root.camZoom / 30);
                    _paintCount++;
                    if (_paintCount === 1 || _paintCount % 60 === 0) {
                        console.log("[PointCloud] paint #" + _paintCount + " points=" + points.length + " extent=" + extent.toFixed(2) + " scaleToFit=" + scaleToFit.toFixed(2) + " view=" + width + "x" + height);
                    }

                    // Center cloud and scale to fit (orthographic so scale doesn't collapse)
                    var cx_cloud = root.cloudCenterX;
                    var cy_cloud = root.cloudCenterY;
                    var cz_cloud = root.cloudCenterZ;

                    var cosY = Math.cos(camYaw);
                    var sinY = Math.sin(camYaw);
                    var cosP = Math.cos(camPitch);
                    var sinP = Math.sin(camPitch);

                    for (var i = 0; i < points.length; i++) {
                        var p = points[i];
                        if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) continue;
                        var x0 = p.x - cx_cloud, y0 = p.y - cy_cloud, z0 = p.z - cz_cloud;
                        var x1 = x0 * cosY - y0 * sinY;
                        var y1 = x0 * sinY + y0 * cosY;
                        var z1 = negateZ ? -z0 : z0;
                        var x2 = x1;
                        var y2 = y1 * cosP - z1 * sinP;
                        var z2 = y1 * sinP + z1 * cosP;
                        // Cull only points far behind camera
                        if (z2 < -extent) continue;

                        var sx = cx + x2 * scaleToFit;
                        var sy = cy - y2 * scaleToFit;
                        var size = Math.max(1.5, 2);

                        var val = Math.floor(255 * (p.intensity !== undefined ? p.intensity : 0.5));
                        // Blue color scheme: light blue to deep blue based on intensity
                        ctx.fillStyle = "rgb(0, " + Math.floor(150 + val*0.4) + ", 255)";
                        ctx.beginPath();
                        ctx.arc(sx, sy, size, 0, 2 * Math.PI);
                        ctx.fill();
                    }

                    // Draw bounding box
                    if (root.cloudBounds) {
                        var b = root.cloudBounds;
                        // 8 corners
                        var corners = [
                            {x: b.minX, y: b.minY, z: b.minZ}, {x: b.maxX, y: b.minY, z: b.minZ},
                            {x: b.maxX, y: b.maxY, z: b.minZ}, {x: b.minX, y: b.maxY, z: b.minZ},
                            {x: b.minX, y: b.minY, z: b.maxZ}, {x: b.maxX, y: b.minY, z: b.maxZ},
                            {x: b.maxX, y: b.maxY, z: b.maxZ}, {x: b.minX, y: b.maxY, z: b.maxZ}
                        ];
                        // Transform corners
                        var screenCorners = corners.map(p => {
                            var x0 = p.x - cx_cloud, y0 = p.y - cy_cloud, z0 = p.z - cz_cloud;
                            var z0v = root.negateZ ? -z0 : z0;
                            var x1 = x0 * cosY - y0 * sinY, y1 = x0 * sinY + y0 * cosY, z1 = z0v;
                            var x2 = x1, y2 = y1 * cosP - z1 * sinP, z2 = y1 * sinP + z1 * cosP;
                            return { sx: cx + x2 * scaleToFit, sy: cy - y2 * scaleToFit, z: z2 };
                        });

                        ctx.strokeStyle = "rgba(0, 255, 255, 0.5)";
                        ctx.lineWidth = 1;
                        ctx.beginPath();
                        // Bottom face (0-1-2-3)
                        ctx.moveTo(screenCorners[0].sx, screenCorners[0].sy);
                        ctx.lineTo(screenCorners[1].sx, screenCorners[1].sy);
                        ctx.lineTo(screenCorners[2].sx, screenCorners[2].sy);
                        ctx.lineTo(screenCorners[3].sx, screenCorners[3].sy);
                        ctx.lineTo(screenCorners[0].sx, screenCorners[0].sy);
                        // Top face (4-5-6-7)
                        ctx.moveTo(screenCorners[4].sx, screenCorners[4].sy);
                        ctx.lineTo(screenCorners[5].sx, screenCorners[5].sy);
                        ctx.lineTo(screenCorners[6].sx, screenCorners[6].sy);
                        ctx.lineTo(screenCorners[7].sx, screenCorners[7].sy);
                        ctx.lineTo(screenCorners[4].sx, screenCorners[4].sy);
                        // Vertical lines
                        ctx.moveTo(screenCorners[0].sx, screenCorners[0].sy); ctx.lineTo(screenCorners[4].sx, screenCorners[4].sy);
                        ctx.moveTo(screenCorners[1].sx, screenCorners[1].sy); ctx.lineTo(screenCorners[5].sx, screenCorners[5].sy);
                        ctx.moveTo(screenCorners[2].sx, screenCorners[2].sy); ctx.lineTo(screenCorners[6].sx, screenCorners[6].sy);
                        ctx.moveTo(screenCorners[3].sx, screenCorners[3].sy); ctx.lineTo(screenCorners[7].sx, screenCorners[7].sy);
                        ctx.stroke();

                        // Dimensions text
                        ctx.fillStyle = "rgba(200, 200, 255, 0.8)";
                        ctx.font = "10px sans-serif";
                        // X dimension (width) - roughly along bottom edge
                        var dimX = (b.maxX - b.minX).toFixed(1) + "m";
                        ctx.fillText(dimX, (screenCorners[0].sx + screenCorners[1].sx)/2, (screenCorners[0].sy + screenCorners[1].sy)/2);
                        // Y dimension (depth)
                        var dimY = (b.maxY - b.minY).toFixed(1) + "m";
                        ctx.fillText(dimY, (screenCorners[1].sx + screenCorners[2].sx)/2, (screenCorners[1].sy + screenCorners[2].sy)/2);
                        // Z dimension (height)
                        var dimZ = (b.maxZ - b.minZ).toFixed(1) + "m";
                        ctx.fillText(dimZ, (screenCorners[0].sx + screenCorners[4].sx)/2, (screenCorners[0].sy + screenCorners[4].sy)/2);
                    }

                    // Draw drone marker
                    if (root.dronePos) {
                        var dp = root.dronePos;
                        var dx0 = dp.x - cx_cloud, dy0 = dp.y - cy_cloud, dz0 = dp.z - cz_cloud;
                        var dz0v = root.negateZ ? -dz0 : dz0;
                        var dx1 = dx0 * cosY - dy0 * sinY, dy1 = dx0 * sinY + dy0 * cosY, dz1 = dz0v;
                        var dx2 = dx1, dy2 = dy1 * cosP - dz1 * sinP, dz2 = dy1 * sinP + dz1 * cosP;
                        var dsx = cx + dx2 * scaleToFit;
                        var dsy = cy - dy2 * scaleToFit;
                        var droneSize = 12;

                        // Outer glow ring
                        ctx.strokeStyle = "rgba(255, 200, 0, 0.35)";
                        ctx.lineWidth = 3;
                        ctx.beginPath();
                        ctx.arc(dsx, dsy, droneSize + 6, 0, 2 * Math.PI);
                        ctx.stroke();

                        // Diamond body
                        ctx.fillStyle = "rgba(255, 180, 0, 0.9)";
                        ctx.beginPath();
                        ctx.moveTo(dsx, dsy - droneSize);
                        ctx.lineTo(dsx + droneSize * 0.7, dsy);
                        ctx.lineTo(dsx, dsy + droneSize);
                        ctx.lineTo(dsx - droneSize * 0.7, dsy);
                        ctx.closePath();
                        ctx.fill();
                        ctx.strokeStyle = "#FFD700";
                        ctx.lineWidth = 2;
                        ctx.stroke();

                        // Heading indicator line
                        var yawRad = dp.yaw * Math.PI / 180.0;
                        var headLen = droneSize + 10;
                        var hx = dsx + Math.sin(yawRad) * headLen;
                        var hy = dsy - Math.cos(yawRad) * headLen;
                        ctx.strokeStyle = "#FF4444";
                        ctx.lineWidth = 2;
                        ctx.beginPath();
                        ctx.moveTo(dsx, dsy);
                        ctx.lineTo(hx, hy);
                        ctx.stroke();
                        // Arrowhead
                        ctx.fillStyle = "#FF4444";
                        ctx.beginPath();
                        var aLen = 6;
                        ctx.moveTo(hx, hy);
                        ctx.lineTo(hx - aLen * Math.sin(yawRad - 0.5), hy + aLen * Math.cos(yawRad - 0.5));
                        ctx.lineTo(hx - aLen * Math.sin(yawRad + 0.5), hy + aLen * Math.cos(yawRad + 0.5));
                        ctx.closePath();
                        ctx.fill();

                        // Label
                        ctx.fillStyle = "#FFD700";
                        ctx.font = "bold 11px sans-serif";
                        ctx.fillText("DRONE", dsx + droneSize + 4, dsy - droneSize);
                        ctx.font = "10px sans-serif";
                        ctx.fillStyle = "rgba(255, 220, 100, 0.85)";
                        ctx.fillText(dp.x.toFixed(1) + ", " + dp.y.toFixed(1) + ", " + dp.z.toFixed(1), dsx + droneSize + 4, dsy - droneSize + 13);
                    }

                    if (root.selectedPoint) {
                        var sp = root.selectedPoint;
                        var x0s = sp.x - cx_cloud, y0s = sp.y - cy_cloud, z0s = sp.z - cz_cloud;
                        var x1s = x0s * cosY - y0s * sinY;
                        var y1s = x0s * sinY + y0s * cosY;
                        var z1s = negateZ ? -z0s : z0s;
                        var x2s = x1s, y2s = y1s * cosP - z1s * sinP, z2s = y1s * sinP + z1s * cosP;
                        if (z2s >= -extent) {
                            var sxS = cx + x2s * scaleToFit;
                            var syS = cy - y2s * scaleToFit;
                            ctx.strokeStyle = "red";
                            ctx.lineWidth = 2;
                            ctx.beginPath();
                            ctx.arc(sxS, syS, 8, 0, 2 * Math.PI);
                            ctx.stroke();
                            ctx.fillStyle = "white";
                            ctx.font = "12px sans-serif";
                            ctx.fillText("TARGET (" + root.distToTarget.toFixed(2) + "m)", sxS + 12, syS);
                        }
                    }
                }
            }

            MouseArea {
                anchors.fill: parent
                property point lastPos
                property bool isDrag: false
                
                onPressed: (mouse)=> { 
                    lastPos = Qt.point(mouse.x, mouse.y) 
                    isDrag = false
                }
                
                onPositionChanged: (mouse)=> {
                    var dx = mouse.x - lastPos.x;
                    var dy = mouse.y - lastPos.y;
                    if (Math.abs(dx) > 2 || Math.abs(dy) > 2) isDrag = true;
                    
                    root.camYaw += dx * 0.01;
                    root.camPitch = Math.max(-1.5, Math.min(1.5, root.camPitch + dy * 0.01));
                    lastPos = Qt.point(mouse.x, mouse.y);
                    canvas.requestPaint();
                }
                
                onClicked: (mouse) => {
                    if (isDrag) return;
                    var mx = mouse.x, my = mouse.y, minDist = 20, bestPoint = null;
                    var cx = parent.width / 2, cy = parent.height / 2;
                    var points = root.pointCloud;
                    if (!points) return;
                    var cx_cloud = root.cloudCenterX, cy_cloud = root.cloudCenterY, cz_cloud = root.cloudCenterZ;
                    var extent = Math.max(root.cloudExtent, 1);
                    var scaleToFit = Math.min(parent.width, parent.height) / (extent * 1.3) * (root.camZoom / 30);
                    var cosY = Math.cos(root.camYaw), sinY = Math.sin(root.camYaw);
                    var cosP = Math.cos(root.camPitch), sinP = Math.sin(root.camPitch);
                    for (var i = 0; i < points.length; i += Math.max(1, Math.floor(points.length / 3000))) {
                        var p = points[i];
                        var x0 = p.x - cx_cloud, y0 = p.y - cy_cloud, z0 = p.z - cz_cloud;
                        var z0v = root.negateZ ? -z0 : z0;
                        var x1 = x0 * cosY - y0 * sinY, y1 = x0 * sinY + y0 * cosY, z1 = z0v;
                        var x2 = x1, y2 = y1 * cosP - z1 * sinP, z2 = y1 * sinP + z1 * cosP;
                        if (z2 < -extent) continue;
                        var sx = cx + x2 * scaleToFit, sy = cy - y2 * scaleToFit;
                        var d = Math.sqrt((sx - mx)*(sx - mx) + (sy - my)*(sy - my));
                        if (d < minDist) { minDist = d; bestPoint = p; }
                    }
                    if (bestPoint) {
                        root.selectedPoint = bestPoint;
                        // Distance from cloud center (or use 0,0,0 for drone)
                        root.distToTarget = Math.sqrt(bestPoint.x*bestPoint.x + bestPoint.y*bestPoint.y + bestPoint.z*bestPoint.z);
                        canvas.requestPaint();
                    } else {
                        root.selectedPoint = null;
                        canvas.requestPaint();
                    }
                }

                onWheel: (wheel)=> {
                    root.camZoom += wheel.angleDelta.y * 0.05;
                    root.camZoom = Math.max(5, Math.min(100, root.camZoom));
                    canvas.requestPaint();
                }
            }

            Rectangle {
                anchors.left: parent.left
                anchors.bottom: parent.bottom
                anchors.margins: 4
                width: zoomColumn.width + 8
                height: zoomColumn.height + 8
                color: "#191919"
                opacity: 0.8
                radius: 3
                border.color: "#3bc2ff"
                border.width: 1
                
                Column {
                    id: zoomColumn
                    x: 4
                    y: 4
                    spacing: 4
                    QGCLabel {
                        text: qsTr("Zoom: %1").arg(root.camZoom.toFixed(1))
                        color: "white"
                    }
                    QGCLabel {
                        visible: !!root.selectedPoint
                        text: root.selectedPoint ? 
                            "Target: " + root.selectedPoint.x.toFixed(1) + ", " + 
                            root.selectedPoint.y.toFixed(1) + ", " + 
                            root.selectedPoint.z.toFixed(1) + " (D: " + root.distToTarget.toFixed(1) + "m)" 
                            : ""
                        color: "#3bc2ff"
                        font.bold: true
                    }
                    Row {
                        spacing: 4
                        QGCButton {
                            text: "+"
                            width: ScreenTools.defaultFontPixelWidth * 3
                            onClicked: {
                                root.camZoom = Math.min(150, root.camZoom + 5)
                                canvas.requestPaint()
                            }
                        }
                        QGCButton {
                            text: "-"
                            width: ScreenTools.defaultFontPixelWidth * 3
                            onClicked: {
                                root.camZoom = Math.max(5, root.camZoom - 5)
                                canvas.requestPaint()
                            }
                        }
                    }
                }
            }
        }
    }
}
