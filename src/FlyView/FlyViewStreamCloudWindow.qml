import QtQuick
import QtQuick.Window
import QGroundControl.FlyView

Window {
    id: root

    title:       qsTr("Point Cloud Viewer")
    width:       1100
    height:      750
    minimumWidth:  700
    minimumHeight: 480
    color:       "#000000"

    flags: Qt.Window | Qt.WindowTitleHint | Qt.WindowCloseButtonHint
           | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint

    FlyViewStreamCloud {
        anchors.fill: parent
    }
}
