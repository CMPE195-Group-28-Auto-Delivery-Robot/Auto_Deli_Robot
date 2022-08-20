var twist;
var cmdVel;
var gpsfix;
var zedimu;
var publishImmidiately = true;
var manager;
var teleop;
var ros;
var joystck_size = 120;
var robot_IP = "192.168.50.233";

function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}

function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}

function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}

function createJoystick( joystic_id ) {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById(joystic_id);
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: joystck_size/2 + 'px' },
            mode: 'static',
            size: joystck_size,
            color: '#0066ff',
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0);
        });
    }
}

function createCAM_with_stick(cam_id, stick_id, path){
    cam = document.getElementById(cam_id);
    cam.src = "http://" + robot_IP + ":8081/stream?topic="+path+"&type=mjpeg&quality=80";
    cam.onload = function () {
        // joystick and keyboard controls will be available only when video is correctly loaded
        createJoystick(stick_id);
    };
}

function createCAM( cam_id, path ){
    cam = document.getElementById(cam_id);
    cam.src = "http://" + robot_IP + ":8081/stream?topic="+path+"&type=mjpeg&quality=80";
}

function initGPSSubscriber(){
    // Init topic object
    gpsfix = new ROSLIB.Topic({
        ros: ros,
        name: '/fix',
        messageType: 'sensor_msgs/NavSatFix'
    });
}

function initIMUSubscriber(){
    // Init topic object
    zedimu = new ROSLIB.Topic({
        ros: ros,
        name: '/zed2/zed_node/imu/data',
        messageType: 'sensor_msgs/Imu'
    });
}

function initMap() {
  // The location of Uluru
  const uluru = { lat: -25.344, lng: 131.031 };
  // The map, centered at Uluru
  const map = new google.maps.Map(document.getElementById("gps_map"), {
    zoom: 4,
    center: uluru,
  });
  // The marker, positioned at Uluru
  const marker = new google.maps.Marker({
    position: uluru,
    map: map,
  });
}

function gps_subscribtion(){
    gpsfix.subscribe(function(message) {
        // gpsfix.unsubscribe();
        gps_ui = document.getElementById("gps_info");
        gps_ui.innerHTML = message.latitude + ", " + message.longitude + ", " + message.altitude;
        // console.log('Received message on ' + gpsfix.name + ': ' + message.latitude + "," + message.longitude + "," + message.altitude);        
    });
}

function sleep(time){
    var timeStamp = new Date().getTime();
    var endTime = timeStamp + time;
    while(true){
        if (new Date().getTime() > endTime){
            return;
        } 
    }
}

function imu_subscribtion(){
    zedimu.subscribe(function(message) {
        // gpsfix.unsubscribe();
        imu_ui = document.getElementById("imu_info");
        imu_ui.innerHTML = message.linear_acceleration.x + ", " + message.linear_acceleration.y + ", " + message.linear_acceleration.z;
        // console.log('Received message on ' + gpsfix.name + ': ' + message.latitude + "," + message.longitude + "," + message.altitude);        
        sleep(1);
    });
}



window.onload = function () {

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    initVelocityPublisher();
    initGPSSubscriber();
    initIMUSubscriber();
    // get handle for video placeholder
    createCAM_with_stick("rgb_cam","joystick","/zed2/zed_node/rgb_raw/image_raw_color");

    createCAM('depth_cam',"/zed2/zed_node/depth/depth_registered");

    // initMap();
    gps_subscribtion();
    imu_subscribtion();
}