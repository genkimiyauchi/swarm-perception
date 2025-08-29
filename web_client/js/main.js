/**
 * @file <client/js/main.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */


/* List of modes that the web client can take. Each mode changes what UI is displayed. */
const Mode = Object.freeze({
  EXPERIMENT: 'experiment',
  DEBUG: 'debug'
})

/* Define function to run after all files are loaded */
var onAllFilesLoaded = function () {

  /* Init KeyboardState */
  window.keyboard = new KeyboardState();
  
  /* Init client signals and flags */
  window.connectFlag = false;
  window.taskFlag = false;
  window.taskCommand = { command: 'task', signal: 'stop' };
  window.requestFlag = false;
  window.requestCommand = { command: 'request', number: 0 };
  window.sendFlag = false;
  window.sendCommand = { command: 'send', number: 0 };
  window.shareTargetFlag = false;
  window.shareTargetCommand = { command: 'share_target', signal: 'false' };

  /* Initial mode */
  window.mode = Mode.EXPERIMENT;

  /* Generate uuid */
  window.client_id = generateRandomUUID();
  window.username = window.client_id.substring(0,8);
  console.log(window.client_id);

  window.target = ''; // Init robot selection to nothing
  window.targetChanged = false;
  window.connected = false; // Connected to a robot
  
  window.target_found = false;
  window.target_received = false;

  window.logLatestTime = 0; // The latest time a log message was received

  /* On Jquery load */
  $(function () {
    /* main panel-layout of the page */
    $('#layout').w2layout({
      name: 'app_layout',
      padding: 4,
      panels: [
        { type: 'top', size: 55, resizable: false },
        { type: 'left', size: "10%", resizable: true, content: 'left', hidden: true },
        { type: 'main', resizable: true, },
        { type: 'right', size: "25%", style: "background-color: #f2f2f2;border:0px", resizable: true, content: 'right' }
      ]
    });
    /* Log layout */
    $().w2layout({
      name: 'log_layout',
      padding: 5,
      panels: [/* {
        type: 'top',
        size: "50%",
        resizable: true,
        // title: "Map",
        // style: "padding:4px 8px;background:white",
        content: '<canvas id="mini-map"></canvas>'
      }, */{
        type: 'main',
        size: "75%",
        resizable: true,
        title: "Log",
        style: "padding:4px 8px;background:white",
        content: '<div id="contentAreaLog" class="clusterize-content"></div>'
      }, {
        type: 'bottom',
        size: "25%",
        title: "LogErr",
        resizable: true,
        style: "padding:4px 8px;background:white",
        content: '<div id="contentAreaLogErr" class="clusterize-content"></div>'
      }]
    });
    /* Toolbar layout */
    $().w2layout({
      name: 'toolbar_layout',
      padding: 5,
      panels: [{
        type: 'top',
        size: "100%",
        resizable: false,
        // style: "padding:4px 8px;background:white",
        // content: '<div id="contentAreaLog" class="clusterize-content"></div>'
      }, /* {
        type: 'main',
        size: "50%",
        resizable: false,
        // style: "padding:4px 8px;background:white",
        // content: '<div id="contentAreaLogErr" class="clusterize-content"></div>'
      } */]
    });

    /* Make them nested */
    w2ui['app_layout'].content('right', w2ui['log_layout']);
    w2ui['app_layout'].content('top', w2ui['toolbar_layout']);

    /* On Threejs panel Resize */
    w2ui['app_layout'].on('resize', function (event) {
      /* When resizing is complete */
      event.onComplete = function () {
        if (window.threejs_panel) {
          onThreejsPanelResize();
        }
      }
    });

    /* Load main logic code sub-files - sequentially */
    /* load threejs scene */
    loadJS("/js/three_scene.js", function () {
      /* Get the panel from layout */
      window.threejs_panel = $("#layout_app_layout_panel_main .w2ui-panel-content")

      /* Setup scene */
      InitializeThreejs(threejs_panel)
    }, true);

    // loadJS("/js/minimap.js", function () {
    //   /* Get the panel from layout */
    //   window.minimap_panel = $("#layout_log_layout_panel_top")

    //   /* Setup mini-map */
    //   InitializeMinimap(minimap_panel)
    //   console.log(window.minimap_panel.width())
    //   console.log(document.getElementById('mini-map'));

    // }, true);

    /* Load websockets and connect to server */
    loadJS("/js/websockets.js", function () {

      /* Add styling for log divs */
      $("#layout_log_layout_panel_main>div.w2ui-panel-content")
        .attr("id", "scrollAreaLog")
        .addClass("clusterize-scroll")

      $("#layout_log_layout_panel_bottom>div.w2ui-panel-content")
        .attr("id", "scrollAreaLogErr")
        .addClass("clusterize-scroll")

      /* Initialize Log objects */
      function tryInitClusterizeLog(retries = 20, delay = 100) {
        const scrollElemLog = document.getElementById('scrollAreaLog');
        const contentElemLog = document.getElementById('contentAreaLog');
        const scrollElemErr = document.getElementById('scrollAreaLogErr');
        const contentElemErr = document.getElementById('contentAreaLogErr');

        let allReady = true;

        if (scrollElemLog && contentElemLog && !window.log_clusterize) {
          window.log_clusterize = new Clusterize({
            show_no_data_row: false,
            scrollId: "scrollAreaLog",
            contentId: 'contentAreaLog'
          });
        } else if (!scrollElemLog || !contentElemLog) {
          allReady = false;
        }

        if (scrollElemErr && contentElemErr && !window.logerr_clusterize) {
          window.logerr_clusterize = new Clusterize({
            show_no_data_row: false,
            scrollId: "scrollAreaLogErr",
            contentId: 'contentAreaLogErr'
          });
        } else if (!scrollElemErr || !contentElemErr) {
          allReady = false;
        }

        if (!allReady && retries > 0) {
          setTimeout(() => tryInitClusterizeLog(retries - 1, delay), delay);
        }
      }

      setTimeout(() => tryInitClusterizeLog(), 0);

      /* List of available modes */
      let dropListMode = "".concat(
        "<select>"
        + "<option value='exp'>Experiment</option>"
        + "<option value='debug'>Debug</option>"
        + "</select>"
      );

      let confirmModeTag = "".concat(
        "<button type='button'>Confirm</button>"
      );

      /* List of robots */

      let dropListRobots = "<select id='robot_selected'><option value=''>Select robot</option>";

      function updateRobotDropdown() {
        let dropListRobots = "<select id='robot_selected'><option value=''>Select robot</option>";
        window.robotIDs.forEach(function(id) {
          dropListRobots += `<option value='${id}'>${id}</option>`;
        });
        dropListRobots += "</select>";

        // Replace the old dropdown or insert if not present
        let $old = $('#robot_selected');
        if ($old.length) {
          $old.replaceWith(dropListRobots);
        } else {
          // If not present, append to toolbar (adjust selector as needed)
          $("#layout_toolbar_layout_panel_top>div.w2ui-panel-content").append(dropListRobots);
        }
      }

      // Expose the function globally so it can be called from websocket.js
      window.updateRobotDropdown = updateRobotDropdown;

      let confirmButtonTag = "".concat(
        "<button type='button'>Confirm</button>"
      );

      // let taskButtonTag = "".concat(
      //   "<button type='button'>START task</button>"
      // );

      /* Add button on top panel */
      $("#layout_toolbar_layout_panel_top>div.w2ui-panel-content")
        .addClass('toolbar-flex-container')
        .append($("<div/>")
          .addClass('toolbar_counter')
          .attr("title", "Step counter")
          .prop("title", "Step counter")//for IE
          .html("{experiment.counter}")
        )
        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
          .attr('id', 'control_divider')
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-step')
          .attr('id', 'step_button')
          .attr("title", "Step experiment")
          .prop("title", "Step experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'step' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-play')
          .attr('id', 'play_button')
          .attr("title", "Play experiment")
          .prop("title", "Play experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'play' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-pause')
          .attr('id', 'pause_button')
          .attr("title", "Pause experiment")
          .prop("title", "Pause experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'pause' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-ff')
          .attr('id', 'ff_button')
          .attr("title", "Fast forward experiment")
          .prop("title", "Fast forward experiment")//for IE
          .click(function () {
            var steps = parseInt($("#ff_steps_input").val());

            if (steps && steps >= 1 && steps <= 1000) {
              $("#ff_steps_input").val(steps)
              window.wsp.sendPacked({ command: 'fastforward', steps: steps })
            } else {
              window.wsp.sendPacked({ command: 'fastforward' })
            }
          })
        )
        .append($("<input/>")
          .attr('type', 'number')
          .attr('id', 'ff_steps_input')
          .attr('min', '1')
          .attr('max', '1000')
          .attr('value', '2')
          .attr("title", "Fast forward steps")
          .prop("title", "Fast forward steps")//for IE
        )
        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )
        // .append($("<div/>")
        //   .addClass('button')
        //   .attr('id', 'stop_button')
        //   .addClass('icon-stop')
        //   .attr("title", "Terminate experiment")
        //   .prop("title", "Terminate experiment")//for IE
        //   .click(function () {
        //     // window.wsp.send('step')
        //   })
        // )
        // .append($("<div/>")
        //   .addClass('button')
        //   .addClass('icon-reset')
        //   .attr('id', 'reset_button')
        //   .attr("title", "Reset experiment")
        //   .prop("title", "Reset experiment")//for IE
        //   .click(function () {
        //     window.wsp.sendPacked({ command: 'pause' })
        //     window.wsp.sendPacked({ command: 'reset' })
        //     location.reload(); // Reload web client
        //   })
        // )
        // /* Divider */
        // .append($("<div/>")
        //   .addClass('toolbar_divider')
        // )
        // .append($("<div/>")
        //   .addClass('button')
        //   .addClass('icon-settings')
        //   .attr('id', 'settings_button')
        //   .attr("title", "Settings")
        //   .prop("title", "Settings")//for IE
        //   .click(function () {
        //   })
        // )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-help')
          .attr("title", "Help")
          .prop("title", "Help")//for IE
          .click(function () {
            $("#HelpModal").w2popup({
              title: 'Help',
              showClose: true,
              height: 300,
              width: 500
            })
            // window.wsp.sendPacked({ command: 'reset' })
          })
        )

        /* Username */
        .append($("<div/>")
          .addClass('toolbar_divider')
          .attr('id', 'name_divider')
        )

        .append($("<div/>")
          .addClass("toolbar_status")
          .attr('id', 'name_label')
          .html("User ID:")
        )

        .append($("<div/>")
          .addClass("toolbar_status")
          .attr('id', 'username_label')
          .html(window.username)
        )

        /* Mode */
        .append($("<div/>")
          .addClass('toolbar_divider')
          .attr('id', 'mode_divider')
        )

        .append($(dropListMode)
          .attr('id', 'mode_selected')
          .attr("title", "Select mode")
          .prop("title", "Select mode")//for IE
        )

        .append($(confirmModeTag)
          .attr('id','button_mode_select')
          .click(function () {
            let e_mode = document.getElementById('mode_selected');
            let selected_mode = e_mode.options[e_mode.selectedIndex].text;
            var mode_param;

            switch(selected_mode) {
              case 'Debug':
                console.log('Load ' + Mode.DEBUG);
                window.mode = Mode.DEBUG;
                mode_param = window.mode;
                break;
              case 'Experiment':
                console.log('Load ' + Mode.EXPERIMENT);
                window.mode = Mode.EXPERIMENT;
                mode_param = 'exp';
                break;
              default:
                console.log('Unrecognised mode selected');
            }

            window.location.search = '?m=' + mode_param;
            
          })
        )

        /* Robot selection */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )

        .append($(dropListRobots)
          .attr('id', 'robot_selected')
          .attr("title", "Select robot")
          .prop("title", "Select robot")//for IE
        )

        .append($(confirmButtonTag)
          .attr('id','button_connect')
          .click(function () {
            // let e_name = document.getElementById('username_label');
            // window.username = e_name.value;
            let e_robot = document.getElementById('robot_selected');
            window.target = e_robot.options[e_robot.selectedIndex].text;
            window.targetChanged = true;

            if(window.target == 'Select robot') {
              window.target = '';
            }

            window.taskFlag = true;
            window.taskCommand = { command: 'task', signal: 'start' };
            window.connectFlag = true;
            window.connectCommand = { command: 'select_robot' };
          })
        )

        .append($("<div/>")
          .addClass("toolbar_status")
          .attr('id', 'connection-status')
          .html("Disconnected")
        )

        /* Divider */
        // .append($("<div/>")
        //   .addClass('toolbar_divider')
        // )

        // .append($(taskButtonTag)
        //   .attr('id','button_task')
        //   .click(function () {
            
        //     if(window.target != '') {
            
        //       if(window.taskCommand['signal'] == 'stop') {
        //         window.taskCommand['signal'] = 'start';
        //       } else if(window.taskCommand['signal'] == 'start') {
        //         window.taskCommand['signal'] = 'stop';  
        //       }

        //       window.taskFlag = true;
        //     }
        //   })
        // )

        /* Spacer */
        .append($("<div/>").addClass('toolbar-spacer'))

        /* Right side of toolbar */
        .append($("<div/>")
          .addClass("toolbar_status")
          .html("{experiment.status}")
        )

      // let requestButtonTag = "".concat(
      //   "<button type='button'>Request</button>"
      // );

      // let sendButtonTag = "".concat(
      //   "<button type='button'>Send</button>"
      // );

      // /* Add button on top panel */
      // $("#layout_toolbar_layout_panel_main>div.w2ui-panel-content")
      //   .addClass('toolbar-flex-container')
      //   .append($("<input/>")
      //     .attr('type', 'number')
      //     .attr('id', 'request_input')
      //     .attr('min', '0')
      //     .attr('max', '1000')
      //     .attr('value', '0')
      //     .attr("title", "Number of robots to request")
      //     .prop("title", "Number of robots to request")//for IE
      //   )

      //   .append($(requestButtonTag)
      //     .attr('id','button_request')
      //     .attr("title", "Request robots")
      //     .prop("title", "Request robots")//for IE
      //     .click(function () {
      //       window.requestCommand['number'] = parseInt($("#request_input").val());
      //       window.requestFlag = true;
      //     })
      //   )

      //   /* Divider */
      //   .append($("<div/>")
      //     .addClass('toolbar_divider')
      //   )

      //   .append($("<input/>")
      //     .attr('type', 'number')
      //     .attr('id', 'send_input')
      //     .attr('min', '0')
      //     .attr('max', '1000')
      //     .attr('value', '0')
      //     .attr("title", "Number of robots to send")
      //     .prop("title", "Number of robots to send")//for IE
      //   )

      //   .append($(sendButtonTag)
      //     .attr('id','button_send')
      //     .attr("title", "Send robots")
      //     .prop("title", "Send robots")//for IE
      //     .click(function () {
      //       window.sendCommand['number'] = parseInt($("#send_input").val());
      //       window.sendFlag = true;
      //     })
      //   )

      /* Set current mode from url param */
      const queryString = window.location.search;
      const urlParams = new URLSearchParams(queryString);

      setTimeout(function() {
        const selectElem = document.getElementById('mode_selected');
        if (selectElem) {
          switch(urlParams.get('m')) {
            case 'debug':
              window.mode = Mode.DEBUG;
              document.getElementById('mode_selected').selectedIndex = 1;
              break;
            case 'exp':
              window.mode = Mode.EXPERIMENT;
              document.getElementById('mode_selected').selectedIndex = 0;
              break;
            default:
              console.log('Unrecognized mode passed in url: ' + urlParams.get('m'));
          }
        }

        console.log("Mode: " + window.mode);

        /* Modify toolbar according to the current mode */
        if(window.mode == Mode.EXPERIMENT) {

          /* Id of the components to hide */
          let ids = [
                      'control_divider',
                      'step_button',
                      'pause_button',
                      'ff_button',
                      'ff_steps_input',
                      'reset_button',
                      // 'name_divider', 
                      // 'name_label', 
                      // 'username_label',
                      'mode_divider',
                      'mode_selected',
                      'button_mode_select',
                      'robot_selected'
                    ];

          /* Hide toolbar components */
          for(const id of ids) {
            var x = document.getElementById(id);
            if(x) {
              x.style.display = 'none';
            }
          }

          /* Hide error log */
          for(panel of w2ui['log_layout'].panels) {
            if(panel.title == 'LogErr') {
              panel.hidden = true;
            }
          }
        }
      }, 0);
        
      // switch(urlParams.get('rs')) {
      //   case '0':
      //     window.request_send_visible = false;
      //     console.log(window.request_send_visible);
      //     break;
      //   case '1':
      //     window.request_send_visible = true;
      //     console.log(window.request_send_visible);
      //     break;
      //   default:
      //     console.log('Unrecognized request-send signal passed in url: ' + urlParams.get('rs'))
      // }

      /* Set robot to connect from url param */
      if(urlParams.get('l')) {
        var robot_found = false;
        switch(urlParams.get('l')) {
          case '1':
            window.target = 'L1';
            robot_found = true;
            break;
          case '2':
            window.target = 'L2';
            robot_found = true;
            break;
          default:
            console.log('Unrecognized robot passed in url: ' + urlParams.get('l'));
        }

        if(robot_found) {
          window.targetChanged = true;
          window.taskFlag = true;
          window.taskCommand = { command: 'task', signal: 'start' };
          window.connectFlag = true;
          window.connectCommand = { command: 'select_robot' };
        }
      }

      /* Set user ID from url param */
      if(urlParams.get('id')) {
        window.username = urlParams.get('id');
        let e_status = document.getElementById('username_label');
        e_status.textContent = window.username;
      }

      console.log("Username: " + window.username);

      window.experiment = {}

      /* Bind data using rivets */
      rivets.bind($('#experiment'), { experiment: window.experiment })

      $("#preloader").fadeOut()
      ConnectWebSockets()
    }, true);
  });
}

/* Load Jquery - sequentially */
loadJS("/js/libs/jquery.min.js", true)
loadJS("/js/libs/w2ui-1.5.rc1.min.js", true) /* Panels */
loadJS("/js/libs/clusterize.min.js", true) /* Better scroll for logs */
loadJS("/js/libs/jquery.contextMenu.min.js", true); /* Right click */

/* Load Websockets code */
loadJS("/js/libs/websocket-as-promised.js", true); /* basic websockets */
loadJS("/js/libs/robust-websocket.js", true); /* auto Reconnect */

/* Load Three.js code */
loadJS("/js/libs/three.min.js", true);
loadJS("/js/libs/OrbitControls.js", true);

loadJS("/js/libs/CSS2DRenderer.js", true);

loadJS("/js/libs/stats.min.js", true);
loadJS("/js/libs/GLTFLoader.js", true);

// From https://www.npmjs.com/package/@seregpie/three.text-sprite
loadJS("/js/libs/three.text-texture.js", true);
loadJS("/js/libs/three.text-sprite.js", true);

// From https://www.npmjs.com/package/three-mesh-ui
loadJS("/js/libs/three-mesh-ui.min.js", true);

/* Load fabric code */
loadJS("/js/libs/fabric.min.js", true);

/* Load keyboard inputs code */
loadJS("/js/libs/KeyboardState.js", true);

/* Load uuid generator */
loadJS("/js/libs/generateRandomUUID.js", true);

/* Start running javascript after all files are loaded */
loadJS("/js/libs/rivets.bundled.min.js", onAllFilesLoaded, true);
