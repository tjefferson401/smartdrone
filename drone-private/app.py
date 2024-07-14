from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_cors import CORS
import json
from utils import parse_payload

import mujoco
import numpy as np
import sys
import time
import mujoco.viewer
import os
import itertools
import math
from scipy.spatial.transform import Rotation as R


from data_controller import DataController
from drone_sim import Drone

env = Drone('drone_swarm.xml')
env.reset()
m = mujoco.MjModel.from_xml_path('drone_swarm.xml')
d = env.data

viewer = mujoco.viewer.launch_passive(m, d)
controller = DataController()

viewer.cam.distance = 10 * viewer.cam.distance  # Increase the camera distance

# Enable wireframe rendering of the entire scene.
# viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
viewer.sync()
start_time = time.time() * 1000

# replace with your own paths
pem ='/Users/yuandedewei/Desktop/Projects/hackathon/pje-hackathon/certs/192.168.3.70.pem'
key = '/Users/yuandedewei/Desktop/Projects/hackathon/pje-hackathon/certs/192.168.3.70-key.pem'

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
CORS(app)  # Apply CORS to your Flask app
socketio = SocketIO(app, cors_allowed_origins="*")  # Configure SocketIO with CORS

@app.route('/')
def index():
    return render_template('index.html')

"""
message data: 
{"acceleration":{"x":3.0373773988634345,"y":0.9922839215405285,"z":2.3292051641106606},"accelerationIncludingGravity":{"x":-2.11362998406291,"y":0.5963069469612091,"z":-6.006303192791342},"rotationRate":{"alpha":-24.86187321092078,"beta":27.454462755695882,"gamma":125.70659907009319},"interval":0.01666666753590107}"""
@socketio.on('message')
def handle_message(data):
    # print('received message: ' + data)
    payload = parse_payload(data)
    
    controller.update(payload)
    # controller.print_current()
    
    ctrl = controller.get_current()
    env.step(ctrl, render=True)
    viewer.sync()
    
def ws_main():
    socketio.run(app, 
                 host='0.0.0.0', 
                 port=5005, 
                 certfile=pem,
                 keyfile=key,
                )

if __name__ == '__main__':
    ws_main()
