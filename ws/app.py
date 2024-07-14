from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_cors import CORS
import json
from utils import parse_payload


# replace with your own paths
pem ='/your/path/to/cert.pem'
key = '/your/path/to/cert-key.pem'

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
    print('received message: ' + data)
    payload = parse_payload(data)
    
def ws_main():
    socketio.run(app, 
                 host='0.0.0.0', 
                 port=5005, 
                 certfile=pem,
                 keyfile=key,
                )

if __name__ == '__main__':
    ws_main()
