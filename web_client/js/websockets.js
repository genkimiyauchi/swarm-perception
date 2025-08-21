/**
 * @file <client/js/websockets.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */

(function (w) {
  var ConnectWebSockets = function () {
    var sockets_api = server + "?broadcasts,logs";

    /* use wss:// for SSL supported */
    if (window.location.protocol == 'https:') {
      sockets_api = "wss://" + sockets_api
    } else {
      sockets_api = "ws://" + sockets_api
    }

    window.experiment.sockets_api = sockets_api

    var loadingPopup = w2popup.open({
      title: 'Connecting to server at..',
      buttons: sockets_api,
      modal: true,
      showClose: false,
      width: 300,     // width in px
      height: 80,
    });

    window.wsp = new window.WebSocketAsPromised(sockets_api, {
      packMessage: data => JSON.stringify(data),
      unpackMessage: data => JSON.parse(data),
      createWebSocket: url => {
        return new RobustWebSocket(url, null, {
          // The number of milliseconds to wait before a connection is considered to have timed out. Defaults to 4 seconds.
          timeout: 2000,
          // A function that given a CloseEvent or an online event (https://developer.mozilla.org/en-US/docs/Online_and_offline_events) and the `RobustWebSocket`,
          // will return the number of milliseconds to wait to reconnect, or a non-Number to not reconnect.
          // see below for more examples; below is the default functionality.
          shouldReconnect: function (event, ws) {
            if (event.code === 1008 || event.code === 1011) return
            return [0, 3000, 10000][ws.attempts]
          },
          // A boolean indicating whether or not to open the connection automatically. Defaults to true, matching native [WebSocket] behavior.
          // You can open the websocket by calling `open()` when you are ready. You can close and re-open the RobustWebSocket instance as much as you wish.
          automaticOpen: true,
          // A boolean indicating whether to disable subscribing to the connectivity events provided by the browser.
          // By default RobustWebSocket instances use connectivity events to avoid triggering reconnection when the browser is offline. This flag is provided in the unlikely event of cases where this may not be desired.
          ignoreConnectivityEvents: false
        })
      }
    });


    wsp.onUnpackedMessage.addListener(data => {
      /* Only if the message is a broadcast message */
      if (data.type == "broadcast") {
        /* Update experiment */
        window.experiment.data = data;
        window.experiment.state = data.state

        /* Reset settings */
        $("#ff_button").removeClass('active')

        switch (data.state) {
          case 'EXPERIMENT_INITIALIZED':
            window.experiment.status = "Initialized";
            break;
          case 'EXPERIMENT_DONE':
            /* disable all buttons */
            $("#layout_app_layout_panel_top .button").addClass("disabled")
            /* Only enable reset button */
            $("#layout_app_layout_panel_top .button.reset-button")
              .removeClass("disabled")

            window.experiment.status = "Done";
            break;
          case 'EXPERIMENT_PAUSED':
            window.experiment.status = "Paused";
            break;
          case 'EXPERIMENT_FAST_FORWARDING':
            window.experiment.status = "Fast Forwarding";
            break;
          case 'EXPERIMENT_PLAYING':
            window.experiment.status = "Playing";
            break;
          default:
            window.experiment.status = "Unknown";
            break;
        }
        $(".button").removeClass('active')

        switch (data.state) {
          case 'EXPERIMENT_PAUSED':
            $("#pause_button").addClass('active')
            break;
          case 'EXPERIMENT_FAST_FORWARDING':
            $("#ff_button").addClass('active')
            break;
          case 'EXPERIMENT_PLAYING':
            $("#play_button").addClass('active')
            break;
          case 'EXPERIMENT_INITIALIZED':
          case 'EXPERIMENT_DONE':
            $("#reset_button").addClass('active')
            break;
          default:
            break;
        }

        /* Pretty print timestep to minutes and seconds (Assumes 1 second = 10 timesteps) */
        const time = data.steps / 10;
        const minutes = Math.floor(time / 60);
        const seconds = Math.floor(time % 60);
        const milliseconds = time % 60 - seconds;
        window.experiment.counter = String(minutes).padStart(2,'0') +':'+ String(seconds).padStart(2,'0') +'.'+ milliseconds.toFixed(1).substring(2);

        /* Check whether this client is in control of a robot */
        var connectionExists = false;
        var robot_id = '';

        if (data.user_data.connections) {
          for (const key in data.user_data.connections){
            if(data.user_data.connections.hasOwnProperty(key)){
              if(data.user_data.connections[key].id == window.client_id) {
                connectionExists = true;
                robot_id = key;
              }
            }
          }
        }

        /* Store the number of robots working on each task */
        if(data.user_data.tasks) {
          window.robot_per_task = data.user_data.tasks;
        }

        let e_status = document.getElementById('connection-status');

        if(connectionExists) {

          window.connected = true;
          // Change color and text of status
          e_status.textContent = 'Connected to '.concat(robot_id);
          e_status.style.color = '#4CAF50';

          /* Change appearance depending on last signal sent */
          // if(window.taskCommand['signal'] == 'stop') {
            
          //   if(window.target != '') {

          //     window.signalButtonText.set({
          //       content: "START",
          //     });

          //     window.signalIndicator.set({
          //       backgroundColor: new THREE.Color( 0xff0000 ),
          //     });
          //   }

          // } else if(window.taskCommand['signal'] == 'start') {

          //   window.signalButtonText.set({
          //     content: "STOP",
          //   });

          //   window.signalIndicator.set({
          //     backgroundColor: new THREE.Color( 0x00ff00 ),
          //   });
          // }
        } else {

          window.connected = false;
          // Change color and text of status
          e_status.textContent = 'Disconnected';
          e_status.style.color = '#000000';

          // if (window.isInitialized) {

          //   window.signalButtonText.set({
          //     content: "-",
          //   });

          //   window.signalIndicator.set({
          //     backgroundColor: new THREE.Color( 0.4, 0.4, 0.4 ),
          //   });

          // }
        }

        /* Current points obtained */
        window.pointsObtained = data.user_data.points;
        // console.log(window.pointsObtained);

        if (!window.isInitialized) {
          window.isInitialized = true;

          /* TODO: calculate best scale */
          var scale__ = data.arena.size.x * 4;
          // Currently we need 50

          initSceneWithScale(scale__);

          /* Start Animation */
          animate();
        }
      } else if (data.type == "log") {
        if (data.messages && data.timestamp > window.logLatestTime) {
          console.log(data);
          window.logLatestTime = data.timestamp;
          
          var log_ = [], logerr_ = [];
          for (let i = 0; i < data.messages.length; i++) {

            /* Pretty print timestep to minutes and seconds (Assumes 1 second = 10 timesteps) */
            const time = data.messages[i].step / 10;
            const minutes = Math.floor(time / 60);
            const seconds = Math.floor(time % 60);
            const milliseconds = time % 60 - seconds;
            const step = String(minutes).padStart(2,'0') +':'+ String(seconds).padStart(2,'0') +'.'+ milliseconds.toFixed(1).substring(2);

            if (data.messages[i].log_type == 'LOG') {

              var text_color = 'rgb(0,0,0)';
              var robot_id = '';
              var message_type = '';
              var message_content = data.messages[i].log_message;

              /* Parse message */
              // Check if first character is {
              if(data.messages[i].log_message.charAt(0) == '{') {

                /* Get robot id */
                message_content = data.messages[i].log_message.split('}')[1];
                robot_id = data.messages[i].log_message.split('}')[0].split('{')[1];

                /* Get message type */
                message_type = data.messages[i].log_message.split('}')[1].split(']')[0].split('[')[1];

                if(message_type == 'REQUEST') {
                  text_color = 'rgb(255,0,0)';
                } else if(message_type == 'SEND') {
                  text_color = 'rgb(0,128,0)';
                }
              }

              if(window.mode == Mode.DEBUG) {

                /* In DEBUG mode */

                log_.unshift("<div class='log'><pre><span class='b'>[t=" +
                  step + "]</span> <span style='color:" + text_color + "'>" +
                  message_content + "</span></pre></div>");

              } else {

                /* Not in DEBUG mode */

                if(data.messages[i].log_message.startsWith('[LOG]') || window.target == '') {

                  /* Print all [LOG] and leader messages */

                  log_.unshift("<div class='log'><pre><span class='b'>[t=" +
                    step + "]</span> <span style='color:" + text_color + "'>" +
                    message_content + "</span></pre></div>");

                } else {

                  /* Print messages from the specific leader that the user is controlling */

                  if(robot_id == window.target && window.target != '') {
                    log_.unshift("<div class='log'><pre><span class='b'>[t=" +
                      step + "]</span> <span style='color:" + text_color + "'>" +
                      message_content + "</span></pre></div>");
                  }
                }
              }
              
            } else {
              logerr_.unshift("<div class='log'><pre><span class='b'>[t=" +
                step + "]</span> " +
                data.messages[i].log_message + "</pre></div>");
            }
          }
          window.log_clusterize.prepend(log_)
          window.logerr_clusterize.prepend(logerr_)
        }
      }

      /* Updating old state, to detect change in state */
      if (window.experiment.old_state != window.experiment.state) {
        /* Maybe the experiment was reset */
        if (window.experiment.state == "EXPERIMENT_INITIALIZED") {
          /* If previously was done, reset button states */
          if (window.experiment.old_state == "EXPERIMENT_DONE") {
            $("#layout_app_layout_panel_top .button").removeClass("disabled")
          }
          setTimeout(() => { // due to some racing issues
            window.log_clusterize.clear()
            window.logerr_clusterize.clear()
          }, 10);
        }

        window.experiment.old_state = window.experiment.state
      }
    });
    wsp.onOpen.addListener(() => {
      console.log('Connection opened')

      /* Close connecting dialog */
      setTimeout(() => {
        loadingPopup.close()
        w2popup.close()
      }, 500);
      window.experiment.connection = "Connected";
      window.experiment.isConnected = true;
    });

    wsp.onClose.addListener(() => {
      console.log('Connection closed')
      setTimeout(() => {
        loadingPopup.close()
        setTimeout(() => {
          $("#disconnectedModal").w2popup({
            title: 'Cannot connect to server',
            modal: true,
            showClose: false,
            height: 100,
          });
        }, 500);
      }, 500);

      window.experiment.connection = "Not Connected";
      window.experiment.isConnected = false;
    });

    wsp.open().catch(e => console.error(e));
  }

  // commonjs
  if (typeof module !== "undefined") {
    module.exports = ConnectWebSockets;
  }
  else {
    w.ConnectWebSockets = ConnectWebSockets;
  }
}(typeof global !== "undefined" ? global : this));