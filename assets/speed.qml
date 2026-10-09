pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Shapes

Rectangle {
    id: root
    width: 500
    height: 300
    color: palette.window

    required property var speedClass // remove for testing if you'd like

    property real speed: speedClass.speed ? speedClass.speed : 0 //replace with value for testing if you'd like
    property real speedAngle: -90 + (root.speed * 180)

    Item {
        id: semiCircleContainer
        width: Math.min(root.width - 40, (root.height - 40) * 2)
        height: (width / 2) + 25
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        clip: true

        Rectangle {
            id: semiCircle
            width: parent.width
            height: parent.width
            radius: width / 2
            border.width: 12
            border.color: '#555555'
            color: (palette.window.hsvValue > 0.5) ? 'black' : 'white'

            anchors.top: parent.top
            anchors.horizontalCenter: parent.horizontalCenter

            Repeater {
                model: 11
                delegate: Item {
                    required property int index
                    anchors.bottom: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.top
                    rotation: -90 + (index * 18)
                    transformOrigin: Item.Bottom
                    z: 1

                    Rectangle {
                        width: 4.5
                        height: 10
                        color: (palette.window.hsvValue > 0.5) ? 'white' : 'black'
                        anchors.top: parent.top
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.topMargin: 20
                    }

                    Text{
                        anchors.horizontalCenter: parent.horizontalCenter
                        color: (palette.window.hsvValue > 0.5) ? 'white' : 'black'
                        anchors.top: parent.top
                        anchors.topMargin: 30
                        font.pixelSize: 15
                        text: index / 10
                    }
                }

            }

            Rectangle {
               anchors.centerIn: parent
               width: Math.min(parent.width, parent.height) * 0.05
               height: width
               radius: width / 2
               color: '#8B0000'
               z: 1
            }

            Shape{
                id: needle
                width: parent.width * 0.01
                height: parent.height * 0.45
                anchors.bottom: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter

                transformOrigin: Item.Bottom
                rotation: root.speedAngle

                ShapePath {
                    strokeColor: '#8B0000'
                    strokeWidth: 0.5
                    fillColor: '#8B0000'

                    startX: 0
                    startY: needle.height

                    PathLine { x: 0; y: 0 }
                    PathLine { x: needle.width; y: 0 }
                    PathLine { x: needle.width; y: needle.height }
                    PathLine { x: 0; y: needle.height }
            }
            }
        }
    }
}