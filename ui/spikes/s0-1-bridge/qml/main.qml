import QtQuick
import QtQuick.Window
import org.drorcontrol.spike

// Spike S0-1 scene: an attitude horizon plus numeric readouts, updated once per frame.
Window {
    width: 1280
    height: 720
    visible: true
    color: "#212121"
    title: "S0-1 bridge spike"

    Telemetry { id: t }

    // One Rust call per rendered frame: loads the snapshot, sets properties (coalesced).
    FrameAnimation {
        running: true
        onTriggered: if (t.tick()) Qt.quit()
    }

    Item {
        id: horizon
        anchors.centerIn: parent
        width: 600; height: 600
        clip: true
        Item {
            anchors.centerIn: parent
            width: parent.width * 2; height: parent.height * 2
            rotation: -t.roll * 57.2958
            Rectangle { width: parent.width; height: parent.height / 2 + t.pitch * 600; color: "#0B5FA5" }
            Rectangle { y: parent.height / 2 + t.pitch * 600; width: parent.width; height: parent.height; color: "#6B4A2B" }
            Repeater {
                model: 19
                Rectangle {
                    x: parent.width / 2 - width / 2
                    y: parent.height / 2 + t.pitch * 600 + (index - 9) * 40
                    width: index % 2 ? 60 : 120; height: 2; color: "#FAFAFA"
                }
            }
        }
    }

    Column {
        x: 20; y: 20; spacing: 6
        Repeater {
            model: [
                "roll " + (t.roll * 57.2958).toFixed(1) + "°",
                "pitch " + (t.pitch * 57.2958).toFixed(1) + "°",
                "yaw " + (t.yaw * 57.2958).toFixed(1) + "°",
                "alt " + t.alt.toFixed(1) + " m",
                "ingest " + (t.msgs_per_sec / 1000).toFixed(1) + " k msg/s",
                "frame p99 " + t.frame_p99_ms.toFixed(2) + " ms",
                "tick p99 " + t.tick_p99_us.toFixed(0) + " µs",
                "dropped " + t.dropped_frames
            ]
            Text { text: modelData; color: "#FAFAFA"; font.family: "monospace"; font.pixelSize: 20 }
        }
    }
}
