/**
 * @file <client/js/three_scene.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */


var camera, cameraOrtho, cameraRobot, controls, renderer, stats, menuRenderer;
var scale;
var uuid2idMap = {};
var selectedEntities = {}

var mouseDownEvent = null

/* Main 3D scene */
var scene = new THREE.Scene();

/* Second scene to display HUD sprites */
var sceneOrtho = new THREE.Scene();

window.isInitialized = false;
window.isLoadingModels = false;

/* Initialize Three.js scene */
THREE.Object3D.DefaultUp.set(0, 0, 1);
scene.background = new THREE.Color(0x007f7f);

let iconGray = '/images/user_icon_gray.png';
let iconBlue = '/images/user_icon_blue.png';
let iconOrange = '/images/user_icon_orange.png';

// compute mouse position in normalized device coordinates
// (-1 to +1) for both directions.
// Used to raycasting against the interactive elements

let objsToTest = [];

const raycaster = new THREE.Raycaster();

const mouse = new THREE.Vector2();
mouse.x = mouse.y = null;

let selectState = false;

window.addEventListener( 'pointermove', ( event )=>{
	mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
	mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
});

window.addEventListener( 'pointerdown', ()=> { selectState = true });

window.addEventListener( 'pointerup', ()=> { selectState = false });

window.addEventListener( 'touchstart', ( event )=> {
	selectState = true;
	mouse.x = ( event.touches[0].clientX / window.innerWidth ) * 2 - 1;
	mouse.y = - ( event.touches[0].clientY / window.innerHeight ) * 2 + 1;
});

window.addEventListener( 'touchend', ()=> {
	selectState = false;
	mouse.x = null;
	mouse.y = null;
});

/* ----------------------- */
var sceneEntities = [];

var InitializeThreejs = function (threejs_panel) {
  var _width = threejs_panel.width();
  var _height = threejs_panel.height();

  /* WebGLRenderer */
  renderer = new THREE.WebGLRenderer({
    antialias: true
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.outputEncoding = THREE.sRGBEncoding;

  /* Add canvas to page */
  renderer.setSize(_width, _height);
  threejs_panel.append(renderer.domElement);

  renderer.autoClear = false; // To allow render overlay

  /* Right click menu renderer */
  menuRenderer = new THREE.CSS2DRenderer();
  menuRenderer.domElement.id = 'dom_menu'
  menuRenderer.domElement.style.backgroundColor = 'rgba(0,0,0,0.4)';
  menuRenderer.domElement.style.display = 'none';

  /* Add canvas, and menu renderers to page */
  menuRenderer.setSize(_width, _height);
  threejs_panel.append(menuRenderer.domElement);

  /* Lights */
  var light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(1, -1, 1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  var light = new THREE.DirectionalLight(0x222222);
  light.position.set(-1, 1, -1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  var light = new THREE.AmbientLight(0x333333);
  light.position.set(0, 0, 1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  /* Add event hander for mouse left click */
  threejs_panel.click(onThreejsPanelMouseClick);

  /* Add event hander for mouse right click */
  threejs_panel.mouseup(onThreejsPanelMouseContextMenu);
  threejs_panel.mousedown(function (event) {
    mouseDownEvent = event
  });
}

function initSceneWithScale(_scale) {
  scale = _scale;

  /* Top-view camera */
  // camera = new THREE.PerspectiveCamera(45, window.threejs_panel.width() / window.threejs_panel.height(), 0.01, scale * 2500);
  camera = new THREE.PerspectiveCamera( 60, window.threejs_panel.width() / window.threejs_panel.height(), 0.1, 1000 );

  camera.position.set(0, -scale * 1, scale * 2);

  camera.layers.enable(0); // enabled by default
  camera.layers.enable(1); // All selectable objects

  /* Orthogonal camera */
  cameraOrtho = new THREE.OrthographicCamera( - window.threejs_panel.width() / 2, window.threejs_panel.width() / 2, window.threejs_panel.height() / 2, - window.threejs_panel.height() / 2, 1, 100 );
  cameraOrtho.position.z = 10;
  
  cameraOrtho.layers.enable(0); // enabled by default
  cameraOrtho.layers.enable(1); // All selectable objects

  /* Robot perspective camera */
  cameraRobot = new THREE.PerspectiveCamera(60, window.threejs_panel.width() / window.threejs_panel.height(), 0.01, scale * 2500);

  cameraRobot.position.set(-scale * 3, 0, scale * 5);

  cameraRobot.layers.enable(0); // enabled by default
  cameraRobot.layers.enable(1); // All selectable objects

  // Controls
  // Possible types: OrbitControls, MapControls

  controls = new THREE.OrbitControls(camera, renderer.domElement);

  controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2;
  controls.screenSpacePanning = false;
  controls.minDistance = scale / 3;
  controls.maxDistance = scale * 2 * Math.max(window.experiment.data.arena.size.y,
    window.experiment.data.arena.size.x);

  // Enable/disable controls based on mode
  function updateCameraControlsByMode() {
    if (controls && typeof controls.enabled !== 'undefined') {
      controls.enabled = (window.mode === Mode.DEBUG);
    }
  }
  // Call once after controls are created
  updateCameraControlsByMode();
  // Expose for global use (e.g., after mode changes)
  window.updateCameraControlsByMode = updateCameraControlsByMode;

  var floor_found = false;
  window.experiment.data.entities.map(function (entity) {
    if (entity.type == "floor") {
      floor_found = true
    }
  })

  /* If no floor entity in experiment */
  if (!floor_found) {
    /* Ground plane */
    var plane_geometry = new THREE.BoxBufferGeometry(
      window.experiment.data.arena.size.x * scale,
      window.experiment.data.arena.size.y * scale,
      0.01
    );

    /* Bring to on top of zero*/
    plane_geometry.translate(0, 0, -0.005 * scale);

    new THREE.TextureLoader().load("/images/ground.png", function (texture) {
      texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
      texture.repeat.set(scale / 2.5, scale / 2.5);

      var material = new THREE.MeshPhongMaterial({
        map: texture
      })
      var plane = new THREE.Mesh(plane_geometry, material);
      plane.layers.set(0);

      scene.add(plane)
    });
  }

  /***************************/



  /* Main Block */

  const mainContainer = new ThreeMeshUI.Block({
    // ref: 'container',
    padding: 0.025,
    fontFamily: '/fonts/Roboto-msdf.json',
    fontTexture: '/fonts/Roboto-msdf.png',
    fontColor: new THREE.Color(0xffffff),
    fontSupersampling: true,
    backgroundOpacity: 0,
    alignItems: 'start',
  });
  mainContainer.position.set( 0, window.threejs_panel.height() / 2 - 40, 0 );
  sceneOrtho.add(mainContainer);

  // /* User Block */

  // const userContainer = new ThreeMeshUI.Block({
  //   width: 300,
  //   height: 97,
  //   margin: 5,
  //   padding: 5,
  //   contentDirection: "row",
  //   backgroundOpacity: 0.8,
  //   borderRadius: [0, 0, 50, 0],
	// 	borderWidth: 3,
	// 	borderOpacity: 1,
  // });
  // mainContainer.add(userContainer);

  //   /* User Icon Block */

  // const userIconContainer = new ThreeMeshUI.Block({
  //   margin: 2,
  //   padding: 0,
  //   borderRadius: 0,
  //   borderWidth: 1,
  //   borderOpacity: 1,
  //   backgroundOpacity: 1,
  //   backgroundColor: new THREE.Color( 0.9, 0.9, 0.9 ),
  // });
  // userContainer.add(userIconContainer);

  // window.userIcon = new ThreeMeshUI.Block({
  //   margin: 0,
  //   padding: 10,
  //   height: 80,
  //   width: 75,
  // });
  // userIconContainer.add(window.userIcon);

  // new THREE.TextureLoader().load(iconGray, (texture) => {
  //   // necessary for backgroundSize: 'contain'
	// 	texture.wrapS = THREE.RepeatWrapping;
	// 	texture.wrapT = THREE.RepeatWrapping;

  //   window.userIcon.set({
  //     backgroundTexture: texture,
  //     backgroundSize: 'stretch',
  //   });
  // });

  //   /* User Info Block */

  // const userInfoContainer = new ThreeMeshUI.Block({
  //   margin: 4,
  //   contentDirection: "column",
  //   alignItems: 'start',
  //   backgroundOpacity: 0,
  //   borderOpacity: 0,
  //   borderWidth: 1,
  //   borderRadius: 0,
  // });
  // userContainer.add(userInfoContainer);

  //   /* User Follower Block */

  // const userTopContainer = new ThreeMeshUI.Block({
  //   contentDirection: "row",
  //   backgroundOpacity: 0,
  //   fontSize: 28,
  // });
  // userInfoContainer.add(userTopContainer);

  //     /* User Name Block */

  // const userNameContainer = new ThreeMeshUI.Block({
  //   width: 95,
  //   height: 40,
  //   margin: 2,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   textAlign: 'left',
  //   backgroundOpacity: 0,
  //   borderRadius: 0,
  // });
  // userTopContainer.add(userNameContainer);

  // window.userName = new ThreeMeshUI.Text({
  //   content: "User -",
  // });

  // userNameContainer.add(window.userName);

  //     /* User Follower Label Block */

  // const userFollowerContainer = new ThreeMeshUI.Block({
  //   contentDirection: "column",
  //   backgroundOpacity: 0,
  // });
  // userTopContainer.add(userFollowerContainer);

  // const userFollowerLabelContainer = new ThreeMeshUI.Block({
  //   width: 100,
  //   height: 20,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  //   borderRadius: 0,
  // });
  // userFollowerContainer.add(userFollowerLabelContainer);

  // userFollowerLabelContainer.add(
	// 	new ThreeMeshUI.Text({
  //     content: "Follower",
  //     fontSize: 16,
	// 	}),
  // );

  //     /* User Follower Count Block */

  // const userFollowerCountContainer = new ThreeMeshUI.Block({
  //   width: 100,
  //   height: 45,
  //   contentDirection: "row",
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  // });
  // userFollowerContainer.add(userFollowerCountContainer);

  // window.numFollowers = new ThreeMeshUI.Text({
  //   fontSize: 40,
  //   content: "-",
  // });

  // // window.numTaskRequire = new ThreeMeshUI.Text({
  // //   content: "-",
  // // });

  // const userFollowerCount = new ThreeMeshUI.Block({
  //   width: 70,
  //   height: 40,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 36,
  //   backgroundOpacity: 0,
  // });
  // userFollowerCountContainer.add(userFollowerCount);
  // userFollowerCount.add(window.numFollowers);

  // // const userFollowerSlashLabel = new ThreeMeshUI.Block({
  // //   width: 15,
  // //   height: 40,
  // //   // margin: 10,
  // //   justifyContent: 'center',
  // //   alignItems: 'center',
  // //   fontSize: 32,
  // //   backgroundOpacity: 0,
  // // });
  // // userFollowerCountContainer.add(userFollowerSlashLabel);
  // // userFollowerSlashLabel.add(
  // //   new ThreeMeshUI.Text({
  // //     content: " / ",
	// // 	}),
  // // );

  // // const userRequiredCount = new ThreeMeshUI.Block({
  // //   width: 70,
  // //   height: 40,
  // //   // margin: 10,
  // //   justifyContent: 'center',
  // //   alignItems: 'center',
  // //   fontSize: 36,
  // //   backgroundOpacity: 0,
  // // });
  // // userFollowerCountContainer.add(userRequiredCount);
  // // userRequiredCount.add(window.numTaskRequire);

  //   /* User Task Block */

  // const userTaskContainer = new ThreeMeshUI.Block({
  //   // margin: 10,
  //   contentDirection: "row",
  //   // alignItems: 'start',
  //   backgroundOpacity: 0,
  // });
  // userInfoContainer.add(userTaskContainer);

  // /* Other User Block */

  // const otherUserContainer = new ThreeMeshUI.Block({
  //   margin: 5,
  //   padding: 5,
  //   contentDirection: "row",
  //   backgroundOpacity: 0.8,
  //   borderRadius: [0, 0, 40, 0],
	// 	borderWidth: 3,
	// 	borderOpacity: 1,
  // });
  // mainContainer.add(otherUserContainer);

  //   /* Other User Icon Block */

  // const otherUserIconContainer = new ThreeMeshUI.Block({
  //   margin: 2,
  //   padding: 0,
  //   borderRadius: 0,
  //   borderWidth: 1,
  //   borderOpacity: 0,
  //   backgroundOpacity: 1,
  //   backgroundColor: new THREE.Color( 0.9, 0.9, 0.9 ),
  // });
  // otherUserContainer.add(otherUserIconContainer);

  // let iconScale = 0.7;

  // window.otherUserIcon = new ThreeMeshUI.Block({
  //   margin: 0,
  //   padding: 10,
  //   height: 80 * iconScale,
  //   width: 75 * iconScale,
  // });
  // otherUserIconContainer.add(window.otherUserIcon);

  // new THREE.TextureLoader().load(iconGray, (texture) => {
  //   // necessary for backgroundSize: 'contain'
	// 	texture.wrapS = THREE.RepeatWrapping;
	// 	texture.wrapT = THREE.RepeatWrapping;

  //   window.otherUserIcon.set({
  //     backgroundTexture: texture,
  //     backgroundSize: 'stretch',
  //   });
  // });

  //   /* Other User Info Block */

  // const otherUserInfoContainer = new ThreeMeshUI.Block({
  //   margin: 2,
  //   contentDirection: "column",
  //   alignItems: 'start',
  //   backgroundOpacity: 0,
  //   borderOpacity: 0,
  //   borderWidth: 1,
  //   borderRadius: 0,
  // });
  // otherUserContainer.add(otherUserInfoContainer);

  //   /* Other User Follower Block */

  // const otherUserTopContainer = new ThreeMeshUI.Block({
  //   contentDirection: "row",
  //   backgroundOpacity: 0,
  //   fontSize: 24,
  // });
  // otherUserInfoContainer.add(otherUserTopContainer);

  //     /* Other User Name Block */

  // const otherUserNameContainer = new ThreeMeshUI.Block({
  //   width: 80,
  //   height: 40,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  //   borderRadius: 0,
  // });
  // otherUserTopContainer.add(otherUserNameContainer);

  // window.otherUserName = new ThreeMeshUI.Text({
  //   content: "User -",
  // }),

  // otherUserNameContainer.add(window.otherUserName);

  //     /* Other User Follower Label Block */

  // const otherUserFollowerContainer = new ThreeMeshUI.Block({
  //   contentDirection: "column",
  //   backgroundOpacity: 0,
  // });
  // otherUserTopContainer.add(otherUserFollowerContainer);

  // const otherUserFollowerLabelContainer = new ThreeMeshUI.Block({
  //   width: 140,
  //   height: 20,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  //   borderRadius: 0,
  // });
  // otherUserFollowerContainer.add(otherUserFollowerLabelContainer);

  // otherUserFollowerLabelContainer.add(
	// 	new ThreeMeshUI.Text({
  //     content: "Follower / Task",
  //     fontSize: 16,
	// 	}),
  // );

  //     /* Other User Follower Count Block */

  // const otherUserFollowerCountContainer = new ThreeMeshUI.Block({
  //   width: 140,
  //   height: 40,
  //   contentDirection: "row",
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 28,
  //   backgroundOpacity: 0,
  // });
  // otherUserFollowerContainer.add(otherUserFollowerCountContainer);

  // window.numOtherFollowers = new ThreeMeshUI.Text({
  //   content: "-",
  // });

  // window.numOtherTaskRequire = new ThreeMeshUI.Text({
  //   content: "-",
  // });

  // const otherUserFollowerCount = new ThreeMeshUI.Block({
  //   width: 50,
  //   height: 40,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 28,
  //   backgroundOpacity: 0,
  // });
  // otherUserFollowerCountContainer.add(otherUserFollowerCount);
  // otherUserFollowerCount.add(window.numOtherFollowers);

  // const otherUserFollowerSlashLabel = new ThreeMeshUI.Block({
  //   width: 15,
  //   height: 40,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 28,
  //   backgroundOpacity: 0,
  // });
  // otherUserFollowerCountContainer.add(otherUserFollowerSlashLabel);
  // otherUserFollowerSlashLabel.add(
  //   new ThreeMeshUI.Text({
  //     content: " / ",
	// 	}),
  // );

  // const otherUserRequiredCount = new ThreeMeshUI.Block({
  //   width: 50,
  //   height: 40,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 28,
  //   backgroundOpacity: 0,
  // });
  // otherUserFollowerCountContainer.add(otherUserRequiredCount);
  // otherUserRequiredCount.add(window.numOtherTaskRequire);

  /* Target Discovery */

  window.targetStatusText = new ThreeMeshUI.Text({
    content: "Find the target",
    fontSize: 24,
    fontColor: new THREE.Color(0xffffff),
  });

  const targetStatusContainer = new ThreeMeshUI.Block({
    width: 300,
    height: 36,
    margin: 5,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundOpacity: 0.3,
    borderRadius: 10,
  });

  targetStatusContainer.add(window.targetStatusText);
  mainContainer.add(targetStatusContainer);
  
  /* Share target location / Move to target */
  const shareTargetContainer = new ThreeMeshUI.Block({
    padding: 0.025,
    height: 50,
    width: 220,
    fontFamily: '/fonts/Roboto-msdf.json',
    fontTexture: '/fonts/Roboto-msdf.png',
    fontColor: new THREE.Color(0xffffff),
    fontSupersampling: true,
    contentDirection: "column", // <-- Change direction to vertical
    backgroundOpacity: 0, // <-- Make background fully transparent
    borderRadius: 0,      // <-- Remove border radius
    borderWidth: 0,       // <-- Remove border
    borderOpacity: 0,     // <-- Remove border
  });

  // Position in the top-right corner (similar logic as sendContainer)
  shareTargetContainer.position.set(
    window.threejs_panel.width() / 2 - 120, // width/2 - half container width
    window.threejs_panel.height() / 2 - 40, // height/2 - half container height
    0
  );

  window.shareTargetButton = new ThreeMeshUI.Block({
    width: 200,
    height: 60,
    margin: 5,
    justifyContent: 'center',
    alignItems: 'center',
    fontSize: 20,
    borderRadius: 15,
    borderOpacity: 1,
    backgroundOpacity: 1,
    isUI: true
  });
  shareTargetButton.add(
    new ThreeMeshUI.Text({
      content: "1. Share Target",
      fontSize: 22,
      fontColor: new THREE.Color(0xffffff),
    })
  );

  // Use the same color states as the request button
  shareTargetButton.setupState({
    state: "idle",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0x999900), // swapped with moveToTargetButton
      backgroundOpacity: 0.8,
      fontColor: new THREE.Color(0xffffff)
    },
  });
  shareTargetButton.setupState({
    state: "hovered",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0xcccc33), // swapped with moveToTargetButton
      backgroundOpacity: 1,
      fontColor: new THREE.Color(0xffffff)
    },
  });
  shareTargetButton.setupState({
    state: "disabled",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0x333333), // keep disabled color
      backgroundOpacity: 0.5,
      fontColor: new THREE.Color(0xaaaaaa)
    },
  });
  shareTargetButton.setupState({
    state: "selected",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0xcccc33), // swapped with moveToTargetButton
      backgroundOpacity: 1,
      fontColor: new THREE.Color(0xffffff)
    },
    onSet: () => {
        console.log("Share target location");
        window.shareTargetFlag = true;
        window.shareTargetSent = true;
    }
  });
  window.shareTargetButton.setState('idle');

  // Add the button to its container
  shareTargetContainer.add(window.shareTargetButton);

  // Add the container to the orthogonal scene
  sceneOrtho.add(shareTargetContainer);
  objsToTest.push(window.shareTargetButton);


  window.moveToTargetButton = new ThreeMeshUI.Block({
    width: 200,
    height: 60,
    margin: 5,
    justifyContent: 'center',
    alignItems: 'center',
    fontSize: 20,
    borderRadius: 15,
    borderOpacity: 1,
    backgroundOpacity: 1,
    isUI: true
  });
  moveToTargetButton.add(
    new ThreeMeshUI.Text({
      content: "2. Move to Target",
      fontSize: 22,
      fontColor: new THREE.Color(0xffffff),
    })
  );

  // Use the same color states as the request button
  moveToTargetButton.setupState({
    state: "idle",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0x990000), // swapped with shareTargetButton
      backgroundOpacity: 0.8,
      fontColor: new THREE.Color(0xffffff)
    },
  });
  moveToTargetButton.setupState({
    state: "hovered",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0xcc3333), // swapped with shareTargetButton
      backgroundOpacity: 1,
      fontColor: new THREE.Color(0xffffff)
    },
  });
  moveToTargetButton.setupState({
    state: "disabled",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0x333333), // keep disabled color
      backgroundOpacity: 0.5,
      fontColor: new THREE.Color(0xaaaaaa)
    },
  });
  moveToTargetButton.setupState({
    state: "selected",
    attributes: {
      offset: 0.035,
      backgroundColor: new THREE.Color(0xcc3333), // swapped with shareTargetButton
      backgroundOpacity: 1,
      fontColor: new THREE.Color(0xffffff)
    },
    onSet: () => {
      console.log("Move to target location");
      window.moveToTargetFlag = true;
      window.moveToTargetSent = true;
    }
  });
  window.moveToTargetButton.setState('idle');

  // Add the button to its container
  shareTargetContainer.add(window.moveToTargetButton);

  // Add the container to the orthogonal scene
  sceneOrtho.add(shareTargetContainer);
  objsToTest.push(window.moveToTargetButton);

  // /*
  // *  Task Container
  // */
  
  // const taskContainer = new ThreeMeshUI.Block({
  //   // ref: 'container',
  //   padding: 0.025,
  //   fontFamily: '/fonts/Roboto-msdf.json',
  //   fontTexture: '/fonts/Roboto-msdf.png',
  //   fontColor: new THREE.Color(0xffffff),
  //   fontSupersampling: true,
  //   backgroundOpacity: 0.2,
  //   borderRadius: 20,
  //   borderOpacity: 0,
  //   alignItems: 'center',
  // });
  // taskContainer.position.set( 20, window.threejs_panel.height() / 2 - 80, 0 );
  // sceneOrtho.add(taskContainer);

  // /* Current points obtained*/
  // const pointsContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   margin: 10,
  //   height: 30,
  //   width: 150,
  //   justifyContent: 'center',
  //   textAlign: 'left',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // taskContainer.add(pointsContainer);

  // window.pointsText = new ThreeMeshUI.Text({
  //   fontColor: new THREE.Color(0xffff00),
  //   fontSize: 24,
  //   content: "Score: 0",
  // });
  // pointsContainer.add(window.pointsText);

  // /* Task Status Container */
  // const taskStatusContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   // height: 100,
  //   // width: 230,
  //   contentDirection: 'row',
  //   justifyContent: 'center',
  //   textAlign: 'left',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  //   alignItems: 'end',
  // });
  // taskContainer.add(taskStatusContainer);

  // /* Number of robots in task */

  // const inTaskContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   // height: 60,
  //   // width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // taskStatusContainer.add(inTaskContainer);

  // const inTaskLabelContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   height: 20,
  //   width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'end',
  //   textAlign: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // inTaskContainer.add(inTaskLabelContainer);

  // const inTaskLabel = new ThreeMeshUI.Text({
  //   content: 'In Task',
  //   fontSize: 20,
  // });
  // inTaskLabelContainer.add(inTaskLabel);

  // const inTaskNumLabelContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   height: 50,
  //   width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   textAlign: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // inTaskContainer.add(inTaskNumLabelContainer);

  // window.inTaskNum = new ThreeMeshUI.Text({
  //   content: '-',
  //   fontSize: 36,
  // });
  // inTaskNumLabelContainer.add(window.inTaskNum);

  // /* Slash label */

  // const slashContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   height: 50,
  //   width: 20,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // taskStatusContainer.add(slashContainer);
  // slashContainer.add(new ThreeMeshUI.Text({
  //   fontSize: 40,
  //   content: '/',
  // }));

  // /* Number of robots required */

  // const requiredContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   // height: 60,
  //   // width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // taskStatusContainer.add(requiredContainer);

  // const requiredLabelContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   height: 20,
  //   width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   textAlign: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // requiredContainer.add(requiredLabelContainer);

  // const requiredLabel = new ThreeMeshUI.Text({
  //   content: 'Required',
  //   fontSize: 20,
  // });
  // requiredLabelContainer.add(requiredLabel);

  // const requiredNumLabelContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   height: 50,
  //   width: 100,
  //   contentDirection: 'column',
  //   justifyContent: 'center',
  //   textAlign: 'center',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 1,
  // });
  // requiredContainer.add(requiredNumLabelContainer);

  // window.requiredNum = new ThreeMeshUI.Text({
  //   content: '-',
  //   fontSize: 36,
  // });
  // requiredNumLabelContainer.add(window.requiredNum);

  // /* Progress Bar Block */

  // const progressContainer = new ThreeMeshUI.Block({
  //   padding: 0.025,
  //   // height: 100,
  //   // width: 230,
  //   contentDirection: 'row',
  //   justifyContent: 'center',
  //   textAlign: 'left',
  //   backgroundOpacity: 0,
  //   borderWidth: 1,
  //   // borderOpacity: 0,
  //   alignItems: 'start',
  // });
  // taskContainer.add(progressContainer);

  // /* Progress Bar background */
  // window.progressBarWidth = 150;
      
  // const progressBarContainer = new ThreeMeshUI.Block({
  //   width: window.progressBarWidth,
  //   height: 20,
  //   margin: 2,
  //   contentDirection: 'row',
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  //   // borderWidth: 1,
  // });
  // progressContainer.add(progressBarContainer);

  // const progressBar = new ThreeMeshUI.Block({
	// 	width: window.progressBarWidth,
	// 	height: 15,
  //   margin: 0,
	// 	padding: 0,
	// 	justifyContent: 'center',
	// 	alignItems: 'start',
  //   backgroundColor: new THREE.Color( 0.4, 0.4, 0.4 ),
  //   backgroundOpacity: 1,
  //   borderWidth: 2,
  //   borderOpacity: 1,
  //   borderRadius: 10,
	// });
  // progressBarContainer.add(progressBar);

  // /* Progress bar foreground */
  // window.progress = new ThreeMeshUI.Block({
  //   width: 0.001,
  //   height: 15,
  //   margin: 0,
  //   padding: 0,
  //   backgroundColor: new THREE.Color( 0, 0.85, 0 ),
  //   backgroundOpacity: 0,
  //   borderOpacity: 0,
  // });
  // progressBar.add(window.progress);

  //     /* User Progress Percent Block */

  // const progressPercentContainer = new ThreeMeshUI.Block({
  //   width: 50,
  //   height: 20,
  //   margin: 2,
  //   justifyContent: 'center',
  //   alignItems: 'end',
  //   textAlign: 'right',
  //   backgroundOpacity: 0,
  //   fontSize: 20,
  // });
  // progressContainer.add(progressPercentContainer);
  // // progressBarContainer.add(requiredContainer);

  // window.numProgress = new ThreeMeshUI.Text({
  //   content: "-",
  // });

  // progressPercentContainer.add(

  //   window.numProgress,

	// 	new ThreeMeshUI.Text({
  //     content: " %",
	// 	}),
  // );

  // /* 
  // *  Send Robot Block 
  // */

  // window.sendContainer = new ThreeMeshUI.Block({
  //   // ref: 'container',
  //   padding: 0.025,
  //   height: 120,
  //   fontFamily: '/fonts/Roboto-msdf.json',
  //   fontTexture: '/fonts/Roboto-msdf.png',
  //   fontColor: new THREE.Color(0xffffff),
  //   fontSupersampling: true,
  //   contentDirection: "row",
  //   // alignItems: 'start',
  //   backgroundOpacity: 0.8,
  //   borderRadius: [0, 0, 0, 30],
	// 	borderWidth: 3,
	// 	borderOpacity: 1,
  // });
  // window.sendContainer.position.set( window.threejs_panel.width() / 2 - 155, window.threejs_panel.height() / 2 - 76, 0 );
  // sceneOrtho.add(window.sendContainer);

  // // Set visibility setting
  // window.sendContainer.visible = window.request_send_visible;

  //   /* Control Block */

  // const sendNumberContainer = new ThreeMeshUI.Block({
  //   margin: 10,
  //   contentDirection: "column",
  //   backgroundOpacity: 0,
  //   borderRadius: 0,
	// 	borderWidth: 1,
	// 	// borderColor: new THREE.Color( 0, 0.5, 1 ),
	// 	borderOpacity: 0,
  // });
  // window.sendContainer.add(sendNumberContainer);

  // /* Label Block */

  // const sendLabelContainer = new ThreeMeshUI.Block({
  //   width: 80,
  //   height: 20,
  //   margin: 0,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   backgroundOpacity: 0,
  //   // borderRadius: [0, 50, 0, 50],
  //   borderWidth: 1,
  //   // borderColor: new THREE.Color( 0, 0.5, 1 ),
  //   borderOpacity: 0,
  // });
  // sendNumberContainer.add(sendLabelContainer);

  // sendLabelContainer.add(
  //   new ThreeMeshUI.Text({
  //     content: "Follower",
  //     fontSize: 16,
  //     // fontColor: new THREE.Color( 0, 0, 0 ),
  //   }),
  // );

  //   /* Count Block */

  // const sendCountContainer = new ThreeMeshUI.Block({
  //   width: 70,
  //   height: 60,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 36,
  //   backgroundOpacity: 0,
  //   borderOpacity: 1,
  // });
  // sendNumberContainer.add(sendCountContainer);

  //   /* Counter */
  // window.robotCount = 0;

  // window.robotCountLabel = new ThreeMeshUI.Text({
  //   content: window.robotCount.toString(),
  //   // fontSize: 28,
  //   // fontColor: new THREE.Color( 0, 0, 0 ),
  // });

  // sendCountContainer.add(window.robotCountLabel);

  //   /* Toggle Block */

  // const sendToggleContainer = new ThreeMeshUI.Block({
  //   margin: 0,
  //   alignItems: 'end',
  //   backgroundOpacity: 0.1,
  //   borderRadius: 0,
  //   borderWidth: 1,
  //   // borderColor: new THREE.Color( 0, 0.5, 1 ),
  //   borderOpacity: 0,
  // });
  // window.sendContainer.add(sendToggleContainer);

  // const hoveredStateAttributes = {
  //   state: "hovered",
  //   attributes: {
  //     offset: 0.035,
  //     backgroundColor: new THREE.Color( 0x999999 ),
  //     backgroundOpacity: 1,
  //     fontColor: new THREE.Color( 0xffffff )
  //   },
  // };

  // const idleStateAttributes = {
  //   state: "idle",
  //   attributes: {
  //     offset: 0.035,
  //     backgroundColor: new THREE.Color( 0x666666 ),
  //     backgroundOpacity: 0.3,
  //     fontColor: new THREE.Color( 0xffffff )
  //   },
  // };

  //     /* Add Block */

  // const sendAddButton = new ThreeMeshUI.Block({
  //   width: 40,
  //   height: 40,
  //   margin: 5,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 32,
  //   borderRadius: 20,
  //   borderOpacity: 1,
  //   // backgroundOpacity: 1,
  // })

  // sendAddButton.add(
  //   new ThreeMeshUI.Text({
  //     content: "+",
  //     // fontColor: new THREE.Color( 1, 1, 1 ),
  //   }),
  // );

  //     /* Subtract Block */

  // const sendSubtractButton = new ThreeMeshUI.Block({
  //   width: 40,
  //   height: 40,
  //   margin: 5,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 36,
  //   borderRadius: 20,
  //   borderOpacity: 1,
  //   // backgroundOpacity: 1,
  // })

  // sendSubtractButton.add(
  //   new ThreeMeshUI.Text({
  //     content: "-",
  //     // fontColor: new THREE.Color( 1, 1, 1 ),
  //   }),
  // );

  // const selectedAttributes = {
  //   offset: 0.02,
  //   backgroundColor: new THREE.Color( 0x777777 ),
  //   fontColor: new THREE.Color( 0x222222 )
  // };

  // sendAddButton.setupState({
  //   state: "selected",
  //   attributes: selectedAttributes,
  //   onSet: ()=> {
  //     console.log("Add Selected");
  //     window.robotCount++;
  //     window.robotCountLabel.set({
  //       content: window.robotCount.toString(),
  //     });
  //   }
  // });
  // sendAddButton.setupState( hoveredStateAttributes );
  // sendAddButton.setupState( idleStateAttributes );

  // sendSubtractButton.setupState({
  //   state: "selected",
  //   attributes: selectedAttributes,
  //   onSet: ()=> {
  //     console.log("Subtract Selected");
  //     if(window.robotCount > 0) {
  //       window.robotCount--;
  //       window.robotCountLabel.set({
  //         content: window.robotCount.toString(),
  //       });
  //     }
  //   }
  // });
  // sendSubtractButton.setupState( hoveredStateAttributes );
  // sendSubtractButton.setupState( idleStateAttributes );

  // sendToggleContainer.add(sendAddButton, sendSubtractButton);
  // objsToTest.push(sendAddButton, sendSubtractButton);

  // /* Confirm Block */

  // const sendConfirmContainer = new ThreeMeshUI.Block({
  //   margin: 10,
  //   alignItems: 'center',
  //   backgroundOpacity: 0.1,
  //   borderRadius: 0,
  //   borderWidth: 1,
  //   // borderColor: new THREE.Color( 0, 0.5, 1 ),
  //   borderOpacity: 0,
  // });
  // window.sendContainer.add(sendConfirmContainer);

  // /* Only add request button if the current mode is NOT direct */
  // if(window.mode != Mode.DIRECT) {

  //   /* Request button */
  //   const sendRequestButton = new ThreeMeshUI.Block({
  //     width: 100,
  //     height: 40,
  //     margin: 5,
  //     justifyContent: 'center',
  //     alignItems: 'center',
  //     fontSize: 20,
  //     borderRadius: 15,
  //     borderOpacity: 1,
  //     // backgroundOpacity: 1,
  //   });
  
  //   sendRequestButton.add(
  //     new ThreeMeshUI.Text({
  //       content: "Request",
  //       // fontColor: new THREE.Color( 1, 1, 1 ),
  //     }),
  //   );
  
  //   const hoveredStateRequestAttributes = {
  //     state: "hovered",
  //     attributes: {
  //       offset: 0.035,
  //       backgroundColor: new THREE.Color( 0xcccc33 ),
  //       backgroundOpacity: 1,
  //       fontColor: new THREE.Color( 0xffffff )
  //     },
  //   };
  
  //   const idleStateRequestAttributes = {
  //     state: "idle",
  //     attributes: {
  //       offset: 0.035,
  //       backgroundColor: new THREE.Color( 0x999900 ),
  //       backgroundOpacity: 0.8,
  //       fontColor: new THREE.Color( 0xffffff )
  //     },
  //   };

  //   sendRequestButton.setupState({
  //     state: "selected",
  //     attributes: selectedAttributes,
  //     onSet: ()=> {
  //       console.log("Request Robots Selected");
  //       if(window.robotCount > 0) {
  //         window.requestFlag = true;
  //         window.requestCommand['number'] = window.robotCount;
  //       }
  //     }
  //   });
  //   sendRequestButton.setupState( hoveredStateRequestAttributes );
  //   sendRequestButton.setupState( idleStateRequestAttributes );
  
  //   sendConfirmContainer.add(sendRequestButton);
  //   objsToTest.push(sendRequestButton);
  // }

  // /* Send button */
  // const sendConfirmButton = new ThreeMeshUI.Block({
  //   width: 80,
  //   height: 40,
  //   margin: 5,
  //   justifyContent: 'center',
  //   alignItems: 'center',
  //   fontSize: 20,
  //   borderRadius: 15,
  //   borderOpacity: 1,
  //   // backgroundOpacity: 1,
  // });

  // /* If mode if direct, make the send button larger */
  // if(window.mode == Mode.DIRECT) {
  //   sendConfirmButton.set({
  //     height: 80,
  //   });
  // }

  // sendConfirmButton.add(
  //   new ThreeMeshUI.Text({
  //     content: "Send",
  //     // fontColor: new THREE.Color( 1, 1, 1 ),
  //   }),
  // );

  // const hoveredStateSendAttributes = {
  //   state: "hovered",
  //   attributes: {
  //     offset: 0.035,
  //     backgroundColor: new THREE.Color( 0xcc3333 ),
  //     backgroundOpacity: 1,
  //     fontColor: new THREE.Color( 0xffffff )
  //   },
  // };

  // const idleStateSendAttributes = {
  //   state: "idle",
  //   attributes: {
  //     offset: 0.035,
  //     backgroundColor: new THREE.Color( 0x990000 ),
  //     backgroundOpacity: 0.8,
  //     fontColor: new THREE.Color( 0xffffff )
  //   },
  // };

  // sendConfirmButton.setupState({
  //   state: "selected",
  //   attributes: selectedAttributes,
  //   onSet: ()=> {
  //     console.log("Send Robots Selected");
  //     if(window.robotCount > 0) {
  //       window.sendFlag = true;
  //       window.sendCommand['number'] = window.robotCount;
  //     }
  //   }
  // });
  // sendConfirmButton.setupState( hoveredStateSendAttributes );
  // sendConfirmButton.setupState( idleStateSendAttributes );

  // sendConfirmContainer.add(sendConfirmButton);
  // objsToTest.push(sendConfirmButton);

}

function cleanUpdateScene() {
  window.isLoadingModels = true;
  /* Remove all meshes */

  for (const key in sceneEntities) {
    if (sceneEntities.hasOwnProperty(key)) {

      const object = scene.getObjectByProperty('uuid', sceneEntities[key].uuid);

      if (object) {
        object.geometry.dispose();
        object.material.dispose();
        scene.remove(object);
      }
    }
  }
  /* reset Map */
  uuid2idMap = {};
  selectedEntities = {}

  var count = window.experiment.data.entities.length
  window.experiment.data.entities.map((entity) => {

    if (entity) { //Neglect Null Entities
      GetEntity(entity, scale, function (entityObject) {
        if (entityObject) {
          var mesh = entityObject.getMesh()

          if (mesh) {
            entityObject.uuid = mesh.uuid

            /* Copy basic properties from Argos entities to threejs objects */
            entityObject.id = entity.id;
            entityObject.type_description = entity.type;

            /* UUID to ID map */
            uuid2idMap[mesh.uuid] = entity.id;

            sceneEntities[entity.id] = entityObject;

            /* Its not an object with "is_movable", so considering it movable(robots) */
            if (typeof entity.is_movable === 'undefined' || entity.is_movable === null) {
              /* Hardcoded floor entity in non-selectable entity */
              if (entity.type == "floor") {
                /* Non selectable */
                mesh.layers.set(0);
              } else {
                /* Add to selectable layer */
                mesh.layers.set(1);
                mesh.traverse(function (child) { child.layers.set(1) })
              }
            } else {
              /* If "is_movable" is true */
              if (entity.is_movable && entity.is_movable === true) {
                /* Add to selectable layer */
                mesh.layers.set(1);
              } else {
                /* Non selectable */
                mesh.layers.set(0);
              }
            }

            scene.add(mesh);
          }
        }

        count--;

        if (count == 0) { // Finished loading all models
          window.isLoadingModels = false;
        }
      });
    } else {
      console.error("Entity is null");
    }
  })
}

function onThreejsPanelResize() {
  var _width = window.threejs_panel.width();
  var _height = window.threejs_panel.height();

  if(camera) {
    camera.aspect = _width / _height;
    camera.updateProjectionMatrix();
  }

  renderer.setSize(_width, _height);
  menuRenderer.setSize(_width, _height);
}

function onThreejsPanelMouseContextMenu(event) {
  event.preventDefault();
  if (!event.originalEvent || event.which != 3) {
    return
  }
  if (mouseDownEvent.offsetX !== event.offsetX ||
    mouseDownEvent.offsetY !== event.offsetY) {
    /* User is panning */
    return
  }

  var contextMenuConfig = {
    selector: '#dom_menu',
    appendTo: '#dom_menu',
    trigger: 'none',
    events: {
      show: function () {
        $("#dom_menu").show()
      },
      hide: function (options) {
        $("#dom_menu").hide()
        $.contextMenu('destroy'); // Delete menu
      }
    },
    position: function (opt, x, y) {
      var origX = x - window.threejs_panel.offset().left - 4
      var origY = y - window.threejs_panel.offset().top - 4

      if (origX + opt.$menu.width() > window.threejs_panel.width()) {
        origX -= opt.$menu.width()
      }
      if (origY + opt.$menu.height() > window.threejs_panel.height()) {
        origY -= opt.$menu.height() + 10
      }

      opt.$menu.css({
        top: origY,
        left: origX
      });
    }
  };

  var mouse = new THREE.Vector2();
  var raycaster = new THREE.Raycaster();
  raycaster.layers.set(1);// Allow to select only in layer 1

  // Convert to -1 to +1
  mouse.x = (event.offsetX / window.threejs_panel.width()) * 2 - 1;
  mouse.y = - (event.offsetY / window.threejs_panel.height()) * 2 + 1;


  // update the picking ray with the camera and mouse position
  raycaster.setFromCamera(mouse, camera);

  // calculate objects intersecting the picking ray 
  // true for recursive(nested)
  var intersects = raycaster.intersectObjects(scene.children, true);

  if (intersects.length > 0) {
    /* Get only the top object */
    var object = intersects[0].object
    /* Get root object, whole parent is Scene */
    while (object.parent.type != "Scene") {
      object = object.parent
    }

    /* Already selected */
    if (selectedEntities[object.uuid]) {
      contextMenuConfig.items = {
        "deselect": {
          name: "Deselect",
          accesskey: 'd',
          callback: function () {
            deselectObject(object)
          }
        },
      };
    } else {
      /* Not selected already */
      contextMenuConfig.items = {
        "select": {
          name: "Select",
          accesskey: 's',
          callback: function () {
            /* deselect all currently selected entities */
            for (const uuid in selectedEntities) {
              if (selectedEntities.hasOwnProperty(uuid)) {
                deselectObjectByUUID(uuid)
              }
            }
            selectObject(object)
          }
        },
      };

    }
  } else { // No items intersects
    /* Only one entity is selected */
    if (Object.keys(selectedEntities).length == 1) {
      var ids = [];

      for (const uuid in selectedEntities) {
        if (selectedEntities.hasOwnProperty(uuid)) {
          if (uuid2idMap[uuid]) { // Found object
            ids.push(uuid2idMap[uuid]);
          }// selected object is no more in scene, dont know what to do...
        }
      }

      contextMenuConfig.items = {
        "move": {
          name: "Move selected here",
          accesskey: 'm',
          callback: function () {
            var pos = get2DProjectedPosition(mouse, sceneEntities[ids[0]]);
            var mesh = sceneEntities[ids[0]].getMesh()

            window.wsp.sendPacked({
              command: 'moveEntity',
              entity_id: ids[0],
              position: {
                x: pos.x,
                y: pos.y,
                z: pos.z
              },
              orientation: {
                x: mesh.quaternion._x,
                y: mesh.quaternion._y,
                z: mesh.quaternion._z,
                w: mesh.quaternion._w
              }
            });
          }
        },
      };
    }
  }

  if (contextMenuConfig.items) {
    $.contextMenu(contextMenuConfig);

    /* Show menu */
    $('#dom_menu').contextMenu({ x: event.clientX, y: event.clientY });
  }
}

function onThreejsPanelMouseClick(event) {
  var mouse = new THREE.Vector2();
  var raycaster = new THREE.Raycaster();
  raycaster.layers.set(1);// Allow to select only in layer 1


  // Convert to -1 to +1
  mouse.x = (event.offsetX / window.threejs_panel.width()) * 2 - 1;
  mouse.y = - (event.offsetY / window.threejs_panel.height()) * 2 + 1;


  // update the picking ray with the camera and mouse position
  raycaster.setFromCamera(mouse, camera);

  // calculate objects intersecting the picking ray // true for recursive(nested)
  var intersects = raycaster.intersectObjects(scene.children, true);

  if (intersects.length > 0) {
    /* Get only the top object */
    var object = intersects[0].object
    /* Get root object, whole parent is Scene */
    while (object.parent.type != "Scene") {
      object = object.parent
    }

    /* Already selected */
    if (selectedEntities[object.uuid]) {
      if (event.shiftKey) {
        deselectObject(object)
      }
    } else {/* Object not already selected */
      if (event.shiftKey) {
        /* deselect all currently selected entities */
        for (const uuid in selectedEntities) {
          if (selectedEntities.hasOwnProperty(uuid)) {
            deselectObjectByUUID(uuid)
          }
        }

        /* Add to selection */
        selectObject(object)
      }
    }
  } else { // No object intersected
    var ids = [];

    for (const uuid in selectedEntities) {
      if (selectedEntities.hasOwnProperty(uuid)) {
        if (uuid2idMap[uuid]) { // Found object
          ids.push(uuid2idMap[uuid]);
        }// selected object is no more in scene, dont know what to do...
      }
    }


    /* Move object if  only control pressed, and one object selected */
    if (event.ctrlKey &&
      !event.altKey &&
      !event.shiftKey &&
      ids.length == 1) {

      var pos = get2DProjectedPosition(mouse, sceneEntities[ids[0]]);
      var mesh = sceneEntities[ids[0]].getMesh()

      window.wsp.sendPacked({
        command: 'moveEntity',
        entity_id: ids[0],
        position: {
          x: pos.x,
          y: pos.y,
          z: pos.z
        },
        orientation: {
          x: mesh.quaternion._x,
          y: mesh.quaternion._y,
          z: mesh.quaternion._z,
          w: mesh.quaternion._w
        }
      });
    }
  }
}

function selectObject(object) {
  if (!selectedEntities[object.uuid]) {
    var boundingBox = new THREE.BoxHelper(object, 0x000000);
    selectedEntities[object.uuid] = boundingBox

    scene.add(boundingBox);
  }
}

function deselectObjectByUUID(uuid) {
  if (selectedEntities[uuid]) {
    /* remove from selection */
    var boundingBox = selectedEntities[uuid]

    boundingBox.geometry.dispose();
    boundingBox.material.dispose();
    scene.remove(boundingBox);

    delete selectedEntities[uuid];
  }
}

function deselectObject(object) {
  deselectObjectByUUID(object.uuid)
}

function get2DProjectedPosition(mouse, object) {
  /* Object's Z plane */
  var z_plane = object.mesh.position.z / scale;

  var mouse_point = new THREE.Vector3();
  mouse_point.x = mouse.x;
  mouse_point.y = mouse.y;
  mouse_point.z = z_plane;

  var pos = new THREE.Vector3();


  /* Get point under the mouse */
  mouse_point.unproject(camera);
  mouse_point.sub(camera.position).normalize();
  var distance = (z_plane - camera.position.z) / mouse_point.z;
  pos.copy(camera.position).add(mouse_point.multiplyScalar(distance));

  /* divide by scale to convert back to units from server */
  pos.divideScalar(scale)

  return pos
}

function animate() {
  requestAnimationFrame(animate);
  ThreeMeshUI.update();
  updateButtons();
  updateCommands();
  controls.update();
  // cameraUpdate();
  render();
  // update_minimap();
}

// Called in the loop, get intersection with either the mouse or the VR controllers,
// then update the buttons states according to result

function updateButtons() {

  /* Adjust mouse position to match UI */

  // console.log(mouse.x);
  // console.log(mouse.y);

  const newMouse = new THREE.Vector2();
  newMouse.copy(mouse);

  const translate = new THREE.Vector2(0.25,0.065);
  newMouse.add(translate);

  const resize = new THREE.Vector2(0.75,0.935);
  newMouse.divide(resize);

  // console.log(newMouse.x);
  // console.log(newMouse.y);

	// Find closest intersecting object

	let intersect;

	if ( newMouse.x !== null && newMouse.y !== null ) {

		raycaster.setFromCamera( newMouse, cameraOrtho );

		intersect = raycast();

	};

	// Update targeted button state (if any)

	if ( intersect && intersect.object.isUI ) {

    // console.log(intersect.object);

    if ( selectState ) {

      // Component.setState internally call component.set with the options you defined in component.setupState
      intersect.object.setState( 'selected' );

    } else {

      // Component.setState internally call component.set with the options you defined in component.setupState
      intersect.object.setState( 'hovered' );

    };
	};

	// Update non-targeted buttons state

	objsToTest.forEach( (obj)=> {

		if ( (!intersect || obj !== intersect.object) && obj.isUI ) {

			// Component.setState internally call component.set with the options you defined in component.setupState
			obj.setState( 'idle' );

		};

	});

  console.log('window.target', window.target, window.target == '');

  /* Disable buttons according to current situation */ 
  if ((!window.target_found && !window.shareTargetSent && !window.moveToTargetSent) ||
      (window.target_found && window.shareTargetSent && window.moveToTargetSent) ||
      window.target == '') {
    shareTargetButton.setState('disabled');
    moveToTargetButton.setState('disabled');
  } else if (!window.shareTargetSent) {
    moveToTargetButton.setState('disabled');
  } else {
    shareTargetButton.setState('disabled');
  }

};

function raycast() {

	return objsToTest.reduce( (closestIntersection, obj)=> {

		const intersection = raycaster.intersectObject( obj, true );

		if ( !intersection[0] ) return closestIntersection

		if ( !closestIntersection || intersection[0].distance < closestIntersection.distance ) {

			intersection[0].object = obj;

			return intersection[0]

		} else {

			return closestIntersection

		};

	}, null );

};

function updateCommands() {

  window.keyboard.update();

  /* Build packet to send */
  var packet = {
                client: window.client_id,
                username: window.username,
                robot: window.target 
               };
  var commands = [];

  /* Check connect command (select_robot) */
  if( window.connectFlag ) {
    commands.push(window.connectCommand);
    if( window.connected ) {
      window.connectFlag = false;
    }
  }

  /* Check if task signal is toggelled (start/stop) */
  if( window.taskFlag ) {
    commands.push(window.taskCommand);
    window.taskFlag = false;
  }

  /* Check for a share target command */
  if( window.shareTargetFlag ) {
    if(window.connected) {
      window.shareTargetCommand['signal'] = 'true';
      commands.push(window.shareTargetCommand);
    } else {
      console.log('Not connected to a robot');
    }
    window.shareTargetFlag = false;
  }

  /* Check for a move to target command */
  if( window.moveToTargetFlag ) {
    if(window.connected) {
      window.moveToTargetCommand['signal'] = 'true';
      commands.push(window.moveToTargetCommand);
    } else {
      console.log('Not connected to a robot');
    }
    window.moveToTargetFlag = false;
  }

  // /* Check for a new request command */
  // if( window.requestFlag ) {
  //   commands.push(window.requestCommand);
  //   window.requestFlag = false;
  //   console.log(packet);

  //   window.robotCount = 0;
  //   window.robotCountLabel.set({
  //     content: window.robotCount.toString(),
  //   });
  // }

  // /* Check for a new send command */
  // if( window.sendFlag ) {
  //   commands.push(window.sendCommand);
  //   window.sendFlag = false;
  //   console.log(packet);

  //   window.robotCount = 0;
  //   window.robotCountLabel.set({
  //     content: window.robotCount.toString(),
  //   });
  // }

  var move_direction = '';

  /* Check movement command */
  if( keyboard.pressed("up") || keyboard.pressed("W") ) {
    move_direction += 'U';
  }

  if( keyboard.pressed("down") || keyboard.pressed("S") ) {
    move_direction += 'D';
  }
  
  if( keyboard.pressed("left") || keyboard.pressed("A") ) {
    move_direction += 'L';
  }
  
  if( keyboard.pressed("right") || keyboard.pressed("D") ) {
    move_direction += 'R';
  }

  if(move_direction) {
    console.log(move_direction);
  }

  let data = {
              command: 'move',
              direction: move_direction
             }
  commands.push(data);

  packet['commands'] = commands;
  window.wsp.sendPacked(packet);
  // console.log(packet);
}


function cameraUpdate() {

  /* Find selected leader position and orientation */
  if(window.target != '') {
    if(sceneEntities.hasOwnProperty(window.target)) {

      // Update camera 
      const relativeCameraOffset = new THREE.Vector3(0,0,3);

      const object = scene.getObjectByProperty('uuid', sceneEntities[window.target].uuid);
      const cameraOffset = relativeCameraOffset.applyMatrix4( object.matrixWorld );
  
      cameraRobot.position.x = cameraOffset.x;
      cameraRobot.position.y = cameraOffset.y;
      cameraRobot.position.z = cameraOffset.z;
  
      const relativeFocusOffset = new THREE.Vector3(20,0,0);
      const focusOffset = relativeFocusOffset.applyMatrix4( object.matrixWorld );

      cameraRobot.lookAt( focusOffset );
    }
  } else {

  }
}


function render() {
  /* Experiment is initialized */
  if (window.experiment &&
    window.experiment.data &&
    window.experiment.data.entities &&
    window.isLoadingModels == false) {

    /* Entities count changed, clean and render again */
    if (window.experiment.data.entities.length != Object.keys(sceneEntities).length) {
      cleanUpdateScene();
      return; // Go to load models, do not update
    }

    /* Call update of each entity */
    // window.experiment.data.entities.map((entity) => {
    //   sceneEntities[entity.id].update(entity, scale);
    // });

    window.experiment.data.entities.map((entity) => {
      if (entity.type == "floor" && window.mode == Mode.EXPERIMENT && !window.target_found) {
        // console.log("FLOOR: " + window.target_found)
      } else {
        sceneEntities[entity.id].update(entity, scale);
      }
    });

    /* Update all bounding boxes */
    for (const uuid in selectedEntities) {
      if (selectedEntities.hasOwnProperty(uuid)) {
        selectedEntities[uuid].update();
      }
    }

    /* Update target discovery */ 
    if (window.target_found && !window.target_received) {
      window.targetStatusText.set({
        content: `You found the target`,
        fontColor: new THREE.Color(0xff0000)
      });
    }
    else if (window.target_found && window.target_received) {
      window.targetStatusText.set({
        content: `A robot shared you the target`,
        fontColor: new THREE.Color(0x00ff00)
      });
    }

    // /* Update points obtained */
    // window.pointsText.set({
    //   content: "Score: " + window.pointsObtained
    // });

    /* Update HUD values */
    if(window.target != '') {

    //   if(sceneEntities.hasOwnProperty(window.target)) {

    //     let num_followers = 0; // TO REMOVE: sceneEntities[window.target].entity.user_data.num_followers;

    //     let taskname = ''; // TO REMOVE: sceneEntities[window.target].entity.user_data.taskname;

    //     var num_robot_in_my_task = 0;
        
    //     let num_task_require = 0; // sceneEntities[window.target].entity.user_data.num_task_require;
    //     let num_task_demand = 0; // sceneEntities[window.target].entity.user_data.num_task_demand;
    //     let num_init_task_demand = 0; // sceneEntities[window.target].entity.user_data.num_init_task_demand;

    //     let num_other_followers = 0; // sceneEntities[window.target].entity.user_data.num_other_followers;
    //     let num_other_task_require = 0; // sceneEntities[window.target].entity.user_data.num_other_task_require;

    //     window.numFollowers.set({
    //       content: num_followers.toString(),
    //     });

    //     if(typeof num_robot_in_my_task !== 'undefined') {
    //       var font_color;
    //       if(num_robot_in_my_task < num_task_require) {
    //         font_color = new THREE.Color(0xffff00);
    //       } else {
    //         font_color = new THREE.Color(0x00ff00);
    //       }
    //       window.inTaskNum.set({
    //         content: num_robot_in_my_task.toString(),
    //         fontColor: font_color,
    //       });
    //     } else {
    //       window.inTaskNum.set({
    //         content: '-',
    //         fontColor: new THREE.Color(0xffffff),
    //       });
    //     }

    //     if(num_task_require == 0) {
    //       window.requiredNum.set({
    //         content: '-',
    //       });
    //     } else {
    //       window.requiredNum.set({
    //         content: num_task_require.toString(),
    //       });
    //     }

    //     if(num_init_task_demand == 0) {
    //       window.numProgress.set({
    //         content: "-",
    //       });

    //       window.progress.set({
    //         width: 0.001,
    //         backgroundOpacity: 0,
    //         backgroundColor: new THREE.Color( 0, 0.85, 0 ),
    //       });
    //     } else {
    //       let num_progress = 1 - num_task_demand / num_init_task_demand;

    //       window.numProgress.set({
    //         content: Math.floor(num_progress.toString() * 100).toString(),
    //       });

    //       if(num_progress != 0) {
    //         window.progress.set({
    //           width: num_progress * window.progressBarWidth,
    //           backgroundOpacity: 1,
    //         });

    //         if(Math.floor(num_progress.toString() * 100) == 100) {
    //           window.progress.set({
    //             backgroundColor: new THREE.Color( 0, 1, 0 ),
    //           });
    //         }
    //       }
    //     }

    //     if(num_other_followers < 0) {
    //       window.numOtherFollowers.set({
    //         content: "-",
    //       });
    //     } else {
    //       window.numOtherFollowers.set({
    //         content: num_other_followers.toString(),
    //       });
    //     }

    //     if(num_other_task_require == 0) {
    //       window.numOtherTaskRequire.set({
    //         content: "-",
    //       });
    //     } else {
    //       window.numOtherTaskRequire.set({
    //         content: num_other_task_require.toString(),
    //       });
    //     }
    //   }
    // } else {

    //   /* Clear values */
    //   window.numFollowers.set({
    //     content: "-",
    //   });

    //   window.requiredNum.set({
    //     content: "-",
    //   });

    //   window.numOtherFollowers.set({
    //     content: "-",
    //   });

    //   window.numOtherTaskRequire.set({
    //     content: "-",
    //   });

    //   window.numProgress.set({
    //     content: "-",
    //   });

    //   window.progress.set({
    //     width: 0.001,
    //     backgroundOpacity: 0,
    //   });

    //   window.robotCount = 0;
    //   window.robotCountLabel.set({
    //     content: window.robotCount.toString(),
    //   });
    }

    // /* Update icon */
    // if(window.targetChanged) {

    //   var icon1 = iconGray;
    //   var icon2 = iconGray;
    //   var user1 = 'User -';
    //   var user2 = 'User -';

    //   if(window.target === "L1") {
    //     icon1 = iconBlue;
    //     icon2 = iconOrange;
    //     user1 = 'User 1';
    //     user2 = 'User 2';
    //   } else if(window.target === "L2") {
    //     icon1 = iconOrange;
    //     icon2 = iconBlue;
    //     user1 = 'User 2';
    //     user2 = 'User 1';
    //   }

    //   new THREE.TextureLoader().load(icon1, (texture) => {
    //     window.userIcon.set({
    //       backgroundTexture: texture,
    //     });
    //   });
    //   new THREE.TextureLoader().load(icon2, (texture) => {
    //     window.otherUserIcon.set({
    //       backgroundTexture: texture,
    //     });
    //   });

    //   window.userName.set({
    //     content: user1,
    //   });
    //   window.otherUserName.set({
    //     content: user2,
    //   });

    //   window.targetChanged = false;
    // }

    // console.log(window.experiment.data);
  }

  // if(window.connected) {
  //   renderer.render(scene, cameraRobot);
  //   menuRenderer.render(scene, cameraRobot);
  // } else if (window.targetChanged) {
  //   console.log('waiting to connect...');
  // } else if ( window.mode == Mode.DEBUG || window.mode == Mode.NOGUI ) {
    renderer.render(scene, camera);
    menuRenderer.render(scene, camera);
  // }

  renderer.clearDepth();

  /* Hide GUI */
  if(window.mode == Mode.NOGUI) { cameraOrtho.position.z = -10; }

  renderer.render( sceneOrtho, cameraOrtho );

}
