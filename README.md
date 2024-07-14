PE Hackathon
# Drone Phone
## a proof of concept single hand controller for a swarm of drones

Instructions for running 
cd into the directory

#### In order to serve the local deployment from https, use mkcert
General:
`brew install mkcert`
`mkcert -install`
`mkcert <your-ip>`

Store the files created via those commands in /certs
Then, in vite.config.js and app.py, add the paths to those files.

#### 
Frontent:
`npm install`
`npm run dev`

Separate Terminal:
Middleware:
`cd ws && pip install requirements.txt && cd ..`


Mujoco simulation code contained in `drone-private` folder


# smartdrone
