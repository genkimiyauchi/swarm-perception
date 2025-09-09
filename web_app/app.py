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
import random
import datetime

# Scenarios
SCENARIO_EXPERIMENT_DIR = "experiments/study2/"

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

# Experiment group
mode = None # 1 or 2
group1_trials = [122, 99, 26, 109, 105, 70, 63, 64, 14, 49, 10, 9, 65, 17, 6, 18, 20, 66, 22, 93]
group2_trials = [115, 71, 107, 62, 121, 51, 7, 34, 94, 50, 100, 40, 85, 119, 95, 60, 36, 21, 82, 23]
trial_order = []

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
    stop_simulation()
    return render_template("start_page.html", session=session)


# Access to "/experimentpage": redirect to experiment_page.html
@app.route("/experimentpage", methods=["GET"])
def experimentpage():
    stop_simulation()
    return render_template("experiment_page.html", session=session, host_ip=ip_addr)


# Access to "/trial1" to "/trial20": render trialX.html
for i in range(1, 21):
    def make_trial_route(trial_num):
        def trial_page():
            stop_simulation()
            return render_template(f"trial{trial_num}.html", session=session, host_ip=ip_addr)
        return trial_page
    endpoint_name = f"trial{i}_page"
    app.add_url_rule(f"/trial{i}", endpoint=endpoint_name, view_func=make_trial_route(i), methods=["GET"])


# Access to "/endpage": redirect to end_page.html
@app.route("/endpage", methods=["GET"])
def endpage():
    stop_simulation()
    return render_template("end_page.html")


# Background process: Start the simulation
@app.route('/background_process_start', methods=['POST'])
def background_process_start():
    scenario = request.get_data().decode('UTF-8')
    print ("Start '{}'".format(scenario))

    global proc_simulation, proc_webclient

    stop_simulation()

    # combine SCENARIO_EXPERIMENT_DIR and scenario filename
    scenario_file = os.path.join(SCENARIO_EXPERIMENT_DIR, "trial{}.argos".format(trial_order[int(scenario)]))
    proc_simulation = SimulationProcess(scenario_file)

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

    parser = argparse.ArgumentParser(description='Run ARGoS simulations.')
    parser.add_argument('-m', '--mode', type=int, choices=[1, 2], required=True,
                       help='Experiment group: 1 or 2.')
    
    args = parser.parse_args()

    if args.mode == 1:
        mode = 1
        print('Mode: 1')
        trial_order = group1_trials
    elif args.mode == 2:
        mode = 2
        print('Mode: 2')
        trial_order = group2_trials

    random.shuffle(trial_order)

    print("Randomized order of trials 1 to 20:")
    print(trial_order)

    filename = "{}.txt".format(datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

    if os.path.basename(os.path.expanduser("~")) == "docker":
        # Save to /home/docker/swarm-perception/results/
        results_dir = os.path.expanduser("~/swarm-perception/results")
    else:
        # Save to ./results/ in the current working directory
        results_dir = os.path.join(os.getcwd(), "results")

    os.makedirs(results_dir, exist_ok=True)
    filename = os.path.join(results_dir, filename)

    print("Saving trial order to {}".format(filename))

    with open(filename, "w") as f:
        f.write("\n".join(map(str, trial_order)))

    app.run(debug=False, host='0.0.0.0')