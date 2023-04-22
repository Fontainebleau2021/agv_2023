 /**
   * Setup all visualization elements when the page is loaded. 
   */
  var ros;
  var viewer;
  var nav;
  var controller; 
  var cloudScan;

  // Connect to ROS.

  ros = new ROSLIB.Ros({
    url:'ws://106.15.233.31:9090/'
  });
  

  function init() {
    // Create the main viewer.
    viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 600,
      height : 600
    });

    //Setup the nav client.

    nav = NAV.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      serverName : '/move_base'
    });
    
    ros.on('error', function(error) {
      console.log("[Rosbridge connect] ERROR:",error);
    });
  
    // Find out exactly when we made a connection.
    ros.on('connection', function() {
      console.log('Connection made!');
    });
  
    ros.on('close', function() {
      console.log('Connection closed.');
    });

  }
 