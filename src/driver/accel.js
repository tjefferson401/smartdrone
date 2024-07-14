// const devOrientationTimer = null
// const devMotionTimer = null
// const mozOrientationTimer = null

// export const phone_log = (msg) => {
//     if(!window.mobileConsoleLog) { return}
//     mobileConsoleLog.innerHTML = msg
// }

// if (window.DeviceOrientationEvent) {
//     window.addEventListener("deviceorientation", function (event) {
//         console.log("here1")
//         if(devOrientationTimer === null) {
//             devOrientationTimer = setTimeout(() => {
//                 phone_log(event)
//             }, 1000)
//         }

//     }, true);
// } else if (window.DeviceMotionEvent) {
//     console.log("here2")

//     window.addEventListener('devicemotion', function (event) {
//         if(devMotionTimer === null) {
//             devMotionTimer = setTimeout(() => {
//                 phone_log(event)
//                 console.log(event)
//             }, 1000)
//         }
        

//     }, true);
// } else {
//     console.log("here3")

//     window.addEventListener("MozOrientation", function (orientation) {
//         if(mozOrientationTimer === null) {
//             mozOrientationTimer = setTimeout(() => {
//                 phone_log(orientation)
//             }, 1000)
//         }
//     }, true);
// }

