import { useEffect, useRef } from "react"



export const useWebsocket = (url, options = {}) => {
    const wsRef = useRef(null)
    useEffect(() => {
        wsRef.current = io(url, {secure: false}); // Adjust the URL to match your server
    
        wsRef.current.on('connect', function() {
            console.log('Connected to the server!');
        });
    
        wsRef.current.on('message', function(data) {
            console.log('Received message:', data);
        });
    
        wsRef.current.on("error", function(err) {
            console.log('Error:', err);
        })
    

        return () => {
            if(wsRef.current) {
                wsRef.current.close()
            }
        }
      }, [])



    const sendData = (data) => {
        if(!wsRef.current) { return }
        wsRef.current.send(data)
    }




    return sendData;
}