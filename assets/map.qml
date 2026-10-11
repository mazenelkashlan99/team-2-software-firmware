pragma ComponentBehavior: Bound
import QtQuick

Rectangle {
    id: root
    width: 400
    height: 400

    color: palette.window

    required property var mapClass

    property real robotX: mapClass.x !== undefined ? mapClass.x : 0.0
    property real robotY: mapClass.y !== undefined ? mapClass.y : 0.0
    property real robotTheta: mapClass.theta !== undefined ? mapClass.theta : 0.0

    property color themeColor: (palette.window.hsvValue > 0.5) ? "black" : "white"

    Rectangle {
        id: gridRect
        anchors.fill: parent
        anchors.leftMargin: 40
        anchors.rightMargin: 20
        anchors.topMargin: 20
        anchors.bottomMargin: 30

        color: "transparent"
        border.color: root.themeColor
        border.width: 2

        Grid {
            anchors.fill: parent
            rows: 4
            columns: 4
            Repeater {
                model: 16
                delegate: Rectangle {
                    width: gridRect.width / 4
                    height: gridRect.height / 4
                    color: "transparent"
                    border.color: root.themeColor
                    border.width: 1
                    opacity: 0.25
                }
            }
        }

        Rectangle {
            id: robot
            width: 16
            height: 16
            radius: 8
            color: "#F44336"

            x: ((root.robotX / 3.2) * gridRect.width) - (width / 2)
            y: gridRect.height - ((root.robotY / 3.2) * gridRect.height) - (height / 2)

            Rectangle {
                width: 14
                height: 2
                color: "white"
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: parent.horizontalCenter
                transformOrigin: Item.Left
                rotation: -root.robotTheta * (180 / Math.PI)
            }
        }
    }

    Repeater {
        model: 5
        delegate: Text {
            required property int index

            text: ((4 - index) * 0.8).toFixed(1)
            color: root.themeColor
            font.pixelSize: 12
            font.bold: true

            x: gridRect.x - width - 10
            y: gridRect.y + (index * (gridRect.height / 4)) - (height / 2)
        }
    }

    Repeater {
        model: 5
        delegate: Text {
            required property int index

            text: (index * 0.8).toFixed(1)
            color: root.themeColor
            font.pixelSize: 12
            font.bold: true

            x: gridRect.x + (index * (gridRect.width / 4)) - (width / 2)
            y: gridRect.y + gridRect.height + 10
        }
    }
}