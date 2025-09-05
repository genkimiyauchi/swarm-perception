# Based on 
# - https://www.yukiyukiponsu.work/entry/python-Flask-btn-page-move
# - https://stackoverflow.com/a/49334973

from flask import Flask, render_template, request, session, redirect, url_for
import socket
from websocket import create_connection
from worker import SimulationProcess, WebClientProcess

import os
import os.path
import argparse
import logging

# Scenarios
SCENARIO_EXPERIMENT = "experiments/target_tracking_webviz.argos"

# Local machine IP address
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ip_addr = s.getsockname()[0]
simulation_link = "{}:8000".format(ip_addr)

# Store IP address
# if(not os.path.isdir('results')):
#     os.mkdir('results')
# f = open('results/ip_address.txt', 'w')
# f.write(ip_addr)
# f.close()

# Currently running simulation scenario
proc_simulation = None
proc_webclient  = None

app = Flask(__name__)

# Based on https://testdriven.io/blog/flask-sessions/
#
# Details on the Secret Key: https://flask.palletsprojects.com/en/2.0.x/config/#SECRET_KEY
# NOTE: The secret key is used to cryptographically-sign the cookies used for storing
#       the session data.
# Command to generate a key:
#     $ python -c 'import secrets; print(secrets.token_hex())'
app.secret_key = '455f3bcd02702ffca86e711f6e176b5983326372b6f04fa5ea66a97bdb3b9e95'

# Disable web access logging in terminal
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
# app.logger.disabled = True
# log.disabled = True


def stop_simulation():
    global proc_simulation, proc_webclient
    if proc_simulation:
        proc_simulation.stop()
    if proc_webclient:
        proc_webclient.stop()


# Access to "/": redirect to start_page.html
@app.route("/")
def default():
    stop_simulation()
    return redirect(url_for('startpage'))


# Access to "/startpage": redirect to start_page.html
@app.route("/startpage", methods=["GET"])
def startpage():
    if 'username' in session:
        stop_simulation()
        return render_template("start_page.html", session=session)
    else:
        return render_template("start_page.html")


# Access to "/experimentpage": redirect to experiment_page.html
@app.route("/experimentpage", methods=["GET"])
def experimentpage():
    if 'username' in session:
        stop_simulation()
        return render_template("experiment_page.html", session=session, host_ip=ip_addr)
    else:
        return redirect(url_for('startpage'))


# Access to "/endpage": redirect to end_page.html
@app.route("/endpage", methods=["GET"])
def endpage():
    if 'username' in session:
        stop_simulation()
        return render_template("end_page.html")
    else:
        return redirect(url_for('startpage'))


# Clear the username stored in the session object
@app.route('/delete')
def delete_username():
    session.pop('username', default=None)
    session.pop('host_ip', default=None)
    return redirect(url_for('startpage'))


# Background process: Start the simulation
@app.route('/background_process_start', methods=['POST'])
def background_process_start():
    scenario = request.get_data().decode('UTF-8')
    print ("Start '{}'".format(scenario))

    global proc_simulation, proc_webclient

    stop_simulation()

    if(scenario == 'experiment'):
        proc_simulation = SimulationProcess(SCENARIO_EXPERIMENT)        
    else:
        print('No scenario provided.')

    proc_simulation.start()
    proc_webclient = WebClientProcess()
    proc_webclient.start()
    return ("nothing")


# Background process: Stop the simulation
@app.route('/background_process_stop')
def background_process_stop():
    print ("Stop")
    stop_simulation()
    return ("nothing")


# Background process: Record the user's ID
@app.route('/background_process_record_id', methods=['POST'])
def background_process_record_id():
    session['username'] = request.get_data().decode('UTF-8')
    print ("Received id: {}".format(session['username']))
    return ("nothing")


# Background process: Record the host IP address
@app.route('/background_process_record_host_ip', methods=['POST'])
def background_process_record_host_ip():
    session['host_ip'] = request.get_data().decode('UTF-8')
    print ("Received host_ip: {}".format(session['host_ip']))
    return ("nothing")


# Check if the local simulation is running
@app.route('/connection_status', methods=['GET'])
def connect_to_server():
    try:
        ws = create_connection("ws://0.0.0.0:3000")
        ws.recv()
        ws.close()
        return "running"
    except ConnectionRefusedError as error:
        # print("Could not connect to local simulation")
        return "terminated"


if __name__ == "__main__":
    app.run(debug=False, host='0.0.0.0')