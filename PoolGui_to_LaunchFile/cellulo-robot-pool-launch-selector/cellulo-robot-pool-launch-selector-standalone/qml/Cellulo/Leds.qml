import QtQuick 2.0

Item {
    PathView {
        id: leds
        width: parent.width
        height: parent.height
        anchors.centerIn: parent
        model: ListModel {
            ListElement { name: "led0" }
            ListElement { name: "led1" }
            ListElement { name: "led2" }
            ListElement { name: "led3" }
            ListElement { name: "led4" }
            ListElement { name: "led5" }
        }
        //use Rectangle to make circles
        delegate: Rectangle {
            id: led
            width: hex.width*13/75
            height: hex.width*13/75
            radius: hex.width*13/(75*2) //resize only works correctly for diagonal resizes of the windoe ie. when width and height are resized evenly
            color:  if(index == 0) robotComm.led0Color;
                    else if(index == 1) robotComm.led1Color;
                    else if(index == 2) robotComm.led2Color;
                    else if(index == 3) robotComm.led3Color;
                    else if(index == 4) robotComm.led4Color;
                    else if(index == 5) robotComm.led5Color;
        }
        path: Path {
            startX: hex.width/2
            startY: hex.height/2 - hex.height/4
            PathArc { x: hex.width/2; y: hex.height/2+hex.height/4;
                      radiusX: hex.width/4; radiusY: hex.height/4; useLargeArc: true }
            PathArc { x: hex.width/2; y: hex.height/2-hex.height/4;
                      radiusX: hex.width/4; radiusY: hex.height/4; useLargeArc: true }
        }
    }
}

