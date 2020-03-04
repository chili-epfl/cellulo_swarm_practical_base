import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.2
import QtQuick.Controls.Private 1.0
import QtQuick.Controls.Styles 1.3
import Qt.labs.platform 1.0

import QMLBluetoothExtras 1.0
import Cellulo 1.0
import Logger 1.0

ApplicationWindow {
    id: window
    visible: true

    title: 'Cellulo ROS Launch Selector'

    property bool mobile: Qt.platform.os === "android"
    minimumWidth: width
    minimumHeight: height

    property var robotListMacAddrSelectors: []
    property var node_type

    Component.onCompleted: {
        if(CelluloCommUtil.testRobotPoolDaemon()){
            if(CelluloCommUtil.startRobotPoolDaemon())
                toast.show("Robot pool daemon started, connecting...");
            else
                toast.show("Robot pool daemon already running, connecting...");
            client.connectToServer();
        }
        else{
            var err = "/usr/local/bin/cellulorobotpoold not found!!!";
            toast.show(err);
            console.log(err);
        }
    }

    function createRobot(macAddr){
        var newRobot = Qt.createQmlObject("import Cellulo 1.0; CelluloBluetooth{}", window);
        newRobot.autoConnect = false;
        newRobot.macAddr = macAddr;
        return newRobot;
    }

    function jsUcfirst(string)
    {
        return string.charAt(0).toUpperCase() + string.slice(1);
    }


    function writeLaunchFile(path, filename){

        logger.log("<launch>\n\n<!-- Define the generic arguments: -->");
        var i;
        for(i=0;i<client.robots.length;i++){
            var macAddrSelector = robotListMacAddrSelectors.itemAt(i).text;
            console.log(macAddrSelector)
            var mac_adr=macAddrSelector.substring(0,2)+"_"+macAddrSelector.substring(3,5)+"_"+macAddrSelector.substring(6,8)+"_"+macAddrSelector.substring(9,11)+"_"+macAddrSelector.substring(12,14)+"_"+macAddrSelector.substring(15,17);
            logger.log("  <arg name=\"mac_adr"+i+"\" default=\""+mac_adr+"\" />");
        }

        logger.log("  <arg name=\"scale\" default=\"1\" />");
        logger.log("  <arg name=\"paper_width\" default=\"500\" />");
        logger.log("  <arg name=\"paper_length\" default=\"500\" />");
        logger.log("  <arg name=\"threshold\" default=\"200\" />");
        logger.log("<!-- End of generic arguments: -->\n");


        if(filename === 'coverage'){
            logger.log("<!-- Define the coverage arguments: -->\n");
            logger.log("  <arg name=\"ko\" default=\"500000\" />");
            logger.log("  <arg name=\"kr\" default=\"500000\" />");
            logger.log("  <arg name=\"m\" default=\"0.2\" />");
            logger.log("  <arg name=\"mu\" default=\"1\" />");
            logger.log("<!-- End of coverage arguments: -->\n");
        }

        if(filename ==='interaction')
        {
            logger.log("<!-- Define the interaction arguments: -->\n");
            logger.log("  <arg name=\"Ku\" default=\"1\" />");
            logger.log("<!-- End of interaction arguments: -->\n");
        }

          if(filename ==='aggregation')
        {
            logger.log("<!-- Define the aggregation arguments: -->\n");
            logger.log(" <!-- <arg name=\"name\" default=\"1\" />  -->\n");
            logger.log("<!-- End of aggregation arguments: -->\n");
        }

        var all_mac_add="";
        //cellulo node
        for(i=0;i<client.robots.length;i++){
            logger.log("    <node name=\"cellulo_node_$(arg mac_adr"+i+")\" pkg=\"ros_cellulo_swarm\" type=\"ros_cellulo_reduced\" output=\"screen\" args=\"$(arg mac_adr"+i+")\">");
            logger.log("        <param name=\"scale_coord\" type=\"double\" value=\"$(arg scale)\" />");
            logger.log("    </node>");
            logger.log("");
            all_mac_add=all_mac_add+"$(arg mac_adr"+i+") ";
        }

        //Sensor node
        for(i=0;i<client.robots.length;i++){
            logger.log("    <node name=\"sensor_node_$(arg mac_adr"+i+")\" pkg=\"ros_cellulo_swarm\" type=\"ros_cellulo_sensor\" output=\"screen\" args=\""+all_mac_add+"\">");
            var j=i+1;
            logger.log("        <param name=\"cellulo_num\" type=\"int\" value=\""+j+"\" />");
            logger.log("        <param name=\"threshold\" type=\"double\" value=\"$(arg threshold)\" />");
            logger.log("        <param name=\"paper_width\" type=\"double\" value=\"$(arg paper_width)\" />");
            logger.log("        <param name=\"paper_length\" type=\"double\" value=\"$(arg paper_length)\" />");
            logger.log("        <param name=\"scale\" type=\"double\" value=\"$(arg scale)\" />");
            logger.log("    </node>");
            logger.log("");

        }

        //task node
        if(filename!=="basic")
        {
            if(filename ==="interaction")
                logger.log("    <node name=\"leader_node\" pkg=\"ros_cellulo_practical\" type=\"ros_cellulo_leader\" output=\"screen\" args=\""+all_mac_add+"\"> </node>");
            for(i=0;i<client.robots.length;i++){
                logger.log("    <node name=\"" + filename +"_node_$(arg mac_adr"+i+")\""+" pkg=\"ros_cellulo_practical\" type=\"ros_cellulo_" + node_type +"\" output=\"screen\" args=\"$(arg mac_adr"+i+")\">");
                if(filename === 'coverage'){
                    logger.log("        <param name=\"ko\" type=\"double\" value=\"$(arg ko)\" />");
                    logger.log("        <param name=\"kr\" type=\"double\" value=\"$(arg kr)\" />");
                    logger.log("        <param name=\"m\" type=\"double\" value=\"$(arg m)\" />");
                    logger.log("        <param name=\"mu\" type=\"double\" value=\"$(arg mu)\" />");
                }
                if(filename === 'aggregation'){
                    logger.log("        <!--<param name=\"name\" type=\"double\" value=\"$(arg name)\" />  -->\n");
                }
                if(filename === 'interaction'){
                    logger.log("        <param name=\"Ku\" type=\"double\" value=\"$(arg Ku)\" />");
                }
                logger.log("        <param name=\"scale\" type=\"double\" value=\"$(arg scale)\" />");
                logger.log("    </node>");
                logger.log("");
            }
        }


        //tf_echo to register the positions
        for(i=0;i<client.robots.length;i++){
        logger.log("    <node pkg=\"tf\" name= \"tf_echo_$(arg mac_adr"+i+")\" type= \"tf_echo\" args=\"paper_world $(arg mac_adr"+i+") 100\" output=\"log\" />\n \n");
        }


        //final adds
        logger.log("    <node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"paper_world_broadcaster\" args=\"0 0 0 0 0 3.1415 base_footprint paper_world\" /> \n");

        logger.log("<!-- launching rviz  -->");
        logger.log("    <arg name=\"model\" default=\"$(find ros_cellulo_swarm)/urdf/cellulo.urdf\"/>");
        logger.log("    <arg name=\"gui\" default=\"true\" />");
        logger.log("    <arg name=\"config_file\" value=\"$(find ros_cellulo_swarm)/rviz/cellulo_rviz.rviz\"/>");
        logger.log("    <node name=\"$(anon rviz)\" pkg=\"rviz\" type=\"rviz\" respawn=\"false\" output=\"screen\" args=\"-d $(arg config_file)\" />");
        logger.log("");
        logger.log("</launch>");
        logger.log(" <!-- THE END  -->");

    }

    CelluloLocalRelayClient{
        id: client

        onConnected: toast.show("Connected to Server.")
        onDisconnected: toast.show("Disconnected from Server.")

        onUnknownRobotAtServer: {
            var robot = createRobot(macAddr);
            addRobot(robot, true);
        }

        function hasRobot(macAddr){
            for(var i=0;i<robots.length;i++)
                if(robots[i].macAddr.toUpperCase() === macAddr.toUpperCase())
                    return true;
            return false;
        }
    }

    Row{
        spacing: 5

        Column{
            spacing: 5

            GroupBox{
                title: "Server controls"

                Row{
                    spacing: 5

                    Button{
                        text: "Start Server"
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: {
                            if(CelluloCommUtil.startRobotPoolDaemon())
                                toast.show("Started robot pool daemon.");
                            else
                                toast.show("Cannot start robot pool daemon, possibly already running.");
                        }
                    }
                    Button{
                        text: "Stop Server"
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: {
                            if(CelluloCommUtil.stopRobotPoolDaemon())
                                toast.show("Stopped robot pool daemon.");
                            else
                                toast.show("Cannot stop robot pool daemon, possibly not running.");
                        }
                    }
                    Text{
                        text: client.connected ? "Connected to Server." : "Connecting to Server..."
                        color: client.connected ? "green" : "red"
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }

            GroupBox{
                title: "Manually add robot"

                Row{
                    spacing: 5

                    Text{
                        id: prefix
                        text: "00:06:66:74:"
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    TextField{
                        id: suffix
                        placeholderText: "XX:XX"
                        anchors.verticalCenter: parent.verticalCenter

                        onAccepted: addButton.clicked()
                    }

                    Button{
                        id: addButton

                        text: "+"
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: {
                            var robot = createRobot(prefix.text + suffix.text);
                            client.addRobot(robot);
                            robotListMacAddrSelectors.push(robot.macAddr);
                        }
                    }
                }
            }

            GroupBox{
                title: "Robots selected in list"

                enabled: client.connected

                Column{
                    id: robotList
                    spacing: 5

                    Repeater{
                        id: robotListMacAddrSelectors

                        model: client.robots.length

                        Label {
                            text: client.robots[index].macAddr
                        }
                    }
                }
            }

            GroupBox{
                title: "Launch File Generator"

                Column{
                    spacing: 5
                    Row{
                        spacing: 20
                        Label {
                            id: type
                            text: 'Type of Launch File'
                        }
                        ComboBox{
                            id: type_of_node
                            width:  150
                            model: [ "basic","interaction", "aggregation", "coverage" ]
                            onCurrentTextChanged:
                                node_type =  type_of_node.currentText
                        }
                    }
                    Row{
                        spacing: 5
                        SimpleLogger{
                            id: logger
                            enabled: true
                            logTime: false
                            logMillis: false
                            logDeviceInfo: false
                            filename: viewer.text.replace('file://','')
                            toConsole: false
                            appendDisabled: true

                        }

                        Button{
                            text: "Select Launch Directory"
                            onClicked:
                                folderDialog.open()
                        }


                        FolderDialog {
                            id: folderDialog
                            currentFolder: viewer.text
                            folder: StandardPaths.standardLocations(StandardPaths.currentPath)[0]
                        }

                        Label {
                            id: viewer
                            text: folderDialog.folder +'/' +node_type +".launch"
                        }


                        Button{
                            text: "Write to launch file"
                            onClicked:
                                writeLaunchFile(viewer.text, node_type)
                        }


                    }
                }
            }
        }

        GroupBox{
            title: "Scan for robots"

            CelluloBluetoothScanner{
                id: scanner
                continuous: continuousCheckBox.checked
            }

            Column{
                id: scanList

                CheckBox{
                    id: continuousCheckBox

                    text: "Scan continuously"
                }

                Row{
                    spacing: 5

                    Button{
                        text: scanner.scanning ? "Scanning..." : "Scan"
                        enabled: !scanner.scanning
                        onClicked: scanner.start()
                    }

                    Button{
                        text: "Stop"
                        enabled: scanner.scanning
                        onClicked: scanner.stop()
                    }
                }

                Repeater{
                    model: scanner.foundRobots.length

                    Row{
                        spacing: 5

                        property string currentMacAddr: scanner.foundRobots[index]

                        Text{
                            text: currentMacAddr
                            anchors.verticalCenter: parent.verticalCenter
                            color: client.hasRobot(currentMacAddr) ? "gray" : "black"
                        }

                        Button{
                            text: "+"
                            anchors.verticalCenter: parent.verticalCenter

                            enabled: !client.hasRobot(currentMacAddr)

                            onClicked: client.addRobot(createRobot(currentMacAddr))
                        }
                    }
                }

                Button{
                    text: "Add all above"
                    enabled: scanner.foundRobots.length > 0
                    onClicked: {
                        for(var i=0;i<scanner.foundRobots.length;i++)
                            if(!client.hasRobot(scanner.foundRobots[i]))
                                client.addRobot(createRobot(scanner.foundRobots[i]));
                    }
                }
            }
        }
    }

    ToastManager{ id: toast }
}
