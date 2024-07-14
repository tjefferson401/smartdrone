import { useEffect, useRef, useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import { useWebsocket } from './comms/websocketHook'
import { VisClient } from "./driver/visualization"
// import { phone_log } from './driver/accel'



function App() {
  const [isSending, setIsSending] = useState(true)
  const [granted, setGranted] = useState(false)
  const VisRef = useRef()
  const sendData = useWebsocket('https://192.168.3.70:5005')

  useEffect(() => {
    VisRef.current = new VisClient()
  }, [])

  useEffect(() => {
    window.isSending = isSending
  }, [isSending])

  const onMotion = (e) => {
      if(!window.isSending ) { return }

      // acceleration should be added to "parent-debugger" div on a rolling basis
      const acceleration = e.acceleration
      const accelerationIncludingGravity = e.accelerationIncludingGravity
      const rotationRate = e.rotationRate
      const interval = e.interval
      // if(VisRef.current) {  
      //   VisRef.current.updateCube(acceleration.x, acceleration.y, acceleration.z, rotationRate.alpha, rotationRate.beta, rotationRate.gamma)
      // }
      const dataObject = {
        acceleration: {
          x: acceleration.x,
          y: acceleration.y,
          z: acceleration.z
        },
        accelerationIncludingGravity: {
          x: accelerationIncludingGravity.x,
          y: accelerationIncludingGravity.y,
          z: accelerationIncludingGravity.z
        },
        rotationRate : {
          alpha: rotationRate.alpha,
          beta: rotationRate.beta,
          gamma: rotationRate.gamma
        },
        interval
      }
      sendData(JSON.stringify(dataObject))
      
  }


  function permission () {
    if ( typeof( DeviceMotionEvent ) !== "undefined" && typeof( DeviceMotionEvent.requestPermission ) === "function" ) {
        // (optional) Do something before API request prompt.
        DeviceMotionEvent.requestPermission()
            .then( response => {
            // (optional) Do something after API prompt dismissed.
            if ( response == "granted" ) {
              setGranted(true)
                window.addEventListener( "devicemotion", onMotion, {
                  frequency: 30
                })
            }
        })
            .catch( console.error )

    } else {
        alert( "DeviceMotionEvent is not defined" );
    }
}


  return (
    <div style={{
      width: "100vw",
      height: "100vh",
      display: "flex",
      justifyContent: "center",
      alignItems: "center"
    }}>
  { !granted &&    <button onClick={permission}
      style={{
        position: "absolute",
        top: 0,
        left: 0,
        zIndex: 100
      }}
    
    >Request Permission</button>}
    <button
      onClick={() => {
        setIsSending(!isSending)
      }}
      style={{
        position: "absolute",
        top: 0,
        right: 0,
        zIndex: 100
      }}
    >
      {isSending ? 'Stop' : 'Start'}
    </button>
    <canvas id="canvas" 

    style={{
      width: "100%",
      height: "100%",
      position: "absolute",
      top: 0,
      left: 0
    }}></canvas>
    
    {/* Live Graph acceleration data */}


    </div>
  )
}

export default App
