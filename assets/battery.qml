pragma ComponentBehavior: Bound
import QtQuick

Item {
    id: root

    width: 100
    height: 40

    required property var batteryClass

    property real level: batteryClass.level ? batteryClass.level : 0.0 //replace with value for testing if you'd like
    property bool isCharging: batteryClass.isCharging ? batteryClass.isCharging : false //replace with value for testing if you'd like

    readonly property real clampedLevel: Math.max(0.0, Math.min(1.0, root.level))

    readonly property color fillColor: {
        if (clampedLevel <= 0.2) return 'red'
        if (clampedLevel <= 0.5) return "#FFC107" // Yellow
        return "#4CAF50"                           // Green
    }

    Rectangle {
        id: batteryBody
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: parent.width * 0.9
        color: "transparent"
        border.width: 3
        border.color: (palette.window.hsvValue > 0.5) ? "black" : "white"
        radius: 6

        Rectangle {
            id: chargeFill
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.margins: 3.5

            width: Math.max(0.0, parent.width * 0.9 * root.clampedLevel)
            color: root.fillColor
            radius: 3

        }

        Text {
            anchors.centerIn: parent
            text: "⚡"
            font.pixelSize: parent.height * 0.5
            color: (palette.window.hsvValue > 0.5) ? "black" : "white"
            visible: root.isCharging
            opacity: 0.8
        }
    }

    Rectangle {
        id: terminal
        anchors.left: batteryBody.right
        anchors.verticalCenter: parent.verticalCenter
        width: parent.width * 0.1
        height: parent.height * 0.4
        color: (palette.window.hsvValue > 0.5) ? "black" : "white"
        radius: 3
    }
}