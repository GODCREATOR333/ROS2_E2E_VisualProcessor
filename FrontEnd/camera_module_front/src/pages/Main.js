import React, { useState, useRef, useEffect } from 'react';
import "./main.css";


function Main() {
    const [isConnected, setIsConnected] = useState(false);
    const videoRef = useRef(null);
    const socketRef = useRef(null);

    useEffect(() => {
        if (isConnected) {
            startVideoStreaming();
            connectToWebSocket();
        } else {
            stopVideoStreaming();
            disconnectFromWebSocket();
        }
    }, [isConnected]);


    const handleConnectClick = () => {
        setIsConnected(true);
    };

    const handleDisconnectClick = () => {
        setIsConnected(false);
        console.log("disconnected");
        if (stopCapture) {
            stopCapture(); // Stop capturing frames
        }
    };

    let stopCapture = null; // Variable to hold the stop function

    const startVideoStreaming = async () => {
        try {
            // Attempt to get user media (camera) stream
            const stream = await navigator.mediaDevices.getUserMedia({
                video: {
                    facingMode: { ideal: "environment" }, // Prefer the back camera
                }
            });
    
            // Set the stream to the video element
            videoRef.current.srcObject = stream;
    
            // Wait for the video metadata to load
            videoRef.current.addEventListener('loadedmetadata', () => {
                // Ensure WebSocket connection is established
                if (socketRef.current && socketRef.current.readyState === WebSocket.OPEN) {
                    // Start capturing frames from the video stream
                    stopCapture = captureFrames(videoRef.current, 30, socketRef.current);
                } else {
                    console.error('WebSocket connection is not open');
                    // Handle the error, e.g., by showing a message to the user
                }
            });
    
            console.log("video stream started");
        } catch (error) {
            console.error('Error accessing camera:', error);
            // Handle errors, e.g., by showing a message to the user
        }
    };
    
    const stopVideoStreaming = () => {
        const stream = videoRef.current.srcObject;
        if (stream) {
            const tracks = stream.getTracks();
            tracks.forEach(track => track.stop());
        }
    };

    const connectToWebSocket = () => {
        
        const socket = new WebSocket("ws://192.168.0.106:9090")

        // Connection opened
        socket.addEventListener("open", event => {
        socket.send("Connection established")
        console.log("Connected")
        socketRef.current = socket;
        });

        // Listen for messages
        socket.addEventListener("message", event => {
        console.log("Message from server ", event.data)
        });

        socket.addEventListener("error", event => {
            console.error("WebSocket error:", event);
        });
        
        socket.addEventListener("close", event => {
            console.log("WebSocket closed:", event.code, event.reason);
        });

        return socket
    };

    const disconnectFromWebSocket = () => {
        // Check if the socketRef is currently holding a WebSocket connection
        if (socketRef.current) {
            // Close the WebSocket connection
            socketRef.current.close();
            // Log a message indicating that the WebSocket has been disconnected
            console.log("Disconnected from WebSocket");
            // Clear the socketRef to indicate that there's no active connection
            socketRef.current = null;
        }
    };

    const captureFrames = (videoElement, frameRate, socket) => {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        const frameInterval = 1000 / frameRate; // Calculate interval between frames
        let isCapturing = true; // Flag to control the capture loop
        let timeoutId = null; // Variable to hold the timeout ID
    
        canvas.width = videoElement.videoWidth;
        canvas.height = videoElement.videoHeight;
    
        const captureFrame = () => {
            if (!isCapturing) return; // Stop capturing if the flag is false
    
            context.drawImage(videoElement, 0, 0, videoElement.videoWidth, videoElement.videoHeight);
            const frameData = canvas.toDataURL('image/jpeg'); // Convert frame to JPEG
    
            // Convert base64 string to byte array
            const byteCharacters = atob(frameData.split(',')[1]);
            const byteArray = new Uint8Array(byteCharacters.length);
            for (let i = 0; i < byteCharacters.length; i++) {
                byteArray[i] = byteCharacters.charCodeAt(i);
            }
    
            if (socket && socket.readyState === WebSocket.OPEN) {
                // Send byte array
                const byteMsg = {
                    op: 'publish',
                    topic: '/camera/image_raw_byte',
                    msg: {
                        data: Array.from(byteArray) // Convert Uint8Array to array for JSON compatibility
                    }
                };
                socket.send(JSON.stringify(byteMsg));
    
                // Send base64 URL
                const base64URL = 'data:image/png;base64,' + btoa(frameData);
                const base64Msg = {
                    op: 'publish',
                    topic: '/camera/image_raw_base64',
                    msg: {
                        data: base64URL
                    }
                };
                socket.send(JSON.stringify(base64Msg));
            } else {
                console.error('WebSocket is not open or is null');
                // Handle the error, e.g., by stopping the capture or retrying
            }
            timeoutId = setTimeout(captureFrame, frameInterval);
        };
    
        captureFrame(); // Start capturing frames
    
        // Return a function to stop capturing frames
        return () => {
            isCapturing = false; // Set the flag to false to stop capturing
            if (timeoutId) {
                clearTimeout(timeoutId); // Clear the timeout
            }
        };
    };
    
    
    

    useEffect(() => {
        return () => {
            if (stopCapture) {
                stopCapture(); // Stop capturing frames when the component unmounts
            }
        };
    }, []);


    return (
        <div className='container'>


            <div className='title'>
                <h1>ROS2 End-to-End Camera Module</h1>
            </div>

            <div className='camera_block'>
                <video ref={videoRef} autoPlay playsInline className="video_feed"></video>
            </div>

            {isConnected ? (
                <div className='btn'>
                    <button className="btn disconnect_btn" onClick={handleDisconnectClick}>Disconnect</button>
                </div>
            ) : (
                <div className='btn'>
                    <button className="btn connect_btn" onClick={handleConnectClick}>Connect</button>
                </div>
            )}
        </div>
    );
}

export default Main;
