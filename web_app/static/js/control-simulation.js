// Call simulation to START
function start_simulation(scenario) {
    $.ajax({ 
        url: '/background_process_start', 
        type: 'POST', 
        data: scenario
    })
}

// Call simulation to STOP
$(function() {
    $('a#stop').on('click', function(e) {
      e.preventDefault()
      $.getJSON('/background_process_stop',
          function(data) {
        //do nothing
      });
      return false;
    });
});

$(function() {
    setInterval(function() {
        $.ajax({
            url: "/connection_status",
            type: "get",
            success: function(response) {
            //   $("#place_for_suggestions").html(response);
                console.log(response);
                if(response === "running") {
                    document.getElementById('simulationState').innerHTML = 'RUNNING';
                    document.getElementById('simulationState').style.color = 'green';
                } else {
                    document.getElementById('simulationState').innerHTML = 'NOT RUNNING';
                    document.getElementById('simulationState').style.color = 'red';
                }
            },
            error: function(xhr) {
              //Do Something to handle error
            }
        });
    }, 250);
});

// Check the local simulation state
// const connect = () => {
//     const ws = new WebSocket("ws://localhost:3000");
//     ws.onopen = () => {
//         console.log("Connection opened!");
//         document.getElementById('simulationState').innerHTML = 'RUNNING';
//         document.getElementById('simulationState').style.color = 'green';
//     };

//     ws.onclose = (e) => {
//         console.log("Closed connection. Reconnecting...");
//         document.getElementById('simulationState').innerHTML = 'NOT RUNNING';
//         document.getElementById('simulationState').style.color = 'red';
//         setTimeout(function () {
//         connect();
//         }, 1000);
//     };

//     ws.onerror = (err) => {
//         console.error(err.message);
//         ws.close();
//     };
// };

// connect();