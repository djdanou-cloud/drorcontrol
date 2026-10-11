// Spike S0-3: one full-window MapLibre item; a fixed camera route set every frame, or manual
// pan / pinch / rotate (trackpad check).
import QtQuick
import MapLibre 4.0

Window {
    id: window

    required property string styleUrl
    required property real passSeconds
    required property bool manual
    required property bool fullscreen

    width: 1600
    height: 1000
    visible: true
    visibility: fullscreen ? Window.FullScreen : Window.Windowed
    title: "DrorControl spike S0-3 (map)"

    property real t0: 0
    property bool running: false

    // Called from C++ at the start of each measured pass.
    function startRoute() {
        t0 = Date.now() / 1000
        running = !manual
    }

    MapLibre {
        id: map
        anchors.fill: parent
        style: window.styleUrl
        // A coastline: land, sea, roads and labels in view (maritime/remote operations).
        coordinate: [32.82, 34.98]
        zoomLevel: 11

        PinchHandler {
            id: pinch
            target: null
            enabled: window.manual
            onScaleChanged: (delta) => map.scale(delta, pinch.centroid.position)
            // Qt rotation is clockwise-positive; turning the map with the fingers lowers the
            // bearing. Whether this feels right is part of the manual check.
            onRotationChanged: (delta) => {
                bearing -= delta
                map.easeTo({ bearing: bearing }, { duration: 0 })
            }
            property real bearing: 0
        }
        DragHandler {
            target: null
            enabled: window.manual
            onTranslationChanged: (delta) => map.pan(delta)
        }
        WheelHandler {
            id: wheel
            enabled: window.manual
            acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
            onWheel: (event) => map.scale(Math.pow(2.0, event.angleDelta.y / 120), wheel.point.position)
        }
    }

    // Route: a figure of eight around the start point, zoom 10–15, a full turn of bearing and
    // tilt up to 45° per pass. The camera changes every frame: the worst case for the renderer.
    FrameAnimation {
        running: window.running
        onTriggered: {
            const s = (Date.now() / 1000 - window.t0) / window.passSeconds   // 0..1 per pass
            const a = 2 * Math.PI * s
            map.easeTo({
                center: [32.82 + 0.25 * Math.sin(a), 34.98 + 0.35 * Math.sin(2 * a)],
                zoom: 12.5 + 2.5 * Math.sin(3 * a),
                bearing: 360 * s,
                pitch: 22.5 - 22.5 * Math.cos(2 * a)
            }, { duration: 0 })
        }
    }

    Text {
        anchors { left: parent.left; bottom: parent.bottom; margins: 8 }
        text: window.manual ? "Manual: pan, pinch and rotate; close the window to finish"
                            : "Measuring: do not touch"
    }
}
