import React, { useState, useEffect } from "react";
import { Joystick } from "react-joystick-component";
import ROSLIB from "roslib";

const JoystickComponent = () => {
    const [ros, setRos] = useState(null);
    const [setConnected] = useState(false);

    useEffect(() => {
        const rosInstance = new ROSLIB.Ros();
        setRos(rosInstance);

        const connectToRos = () => {
            try {
                rosInstance.connect(import.meta.env.VITE_ROS_URL);
            } catch (error) {
                console.log("Connection Problem", error);
            }
        };

        rosInstance.on("connection", () => {
            console.log("Connected to ROS");
            setConnected(true);
        });

        rosInstance.on("close", () => {
            console.log("Disconnected from ROS");
            setConnected(false);
        });

        connectToRos();

        return () => {
            rosInstance.close();
        };
    }, []);

    const handleMove = (event) => {
        if (!ros) return;

        const cmdVel = new ROSLIB.Topic({
            ros,
            name: "/cmd_vel",
            messageType: "geometry_msgs/Twist",
        });

        const twist = new ROSLIB.Message({
            linear: { x: event.y / 3, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: -event.x },
        });

        cmdVel.publish(twist);
    };

    const handleStop = () => {
        if (!ros) return;

        const cmdVel = new ROSLIB.Topic({
            ros,
            name: "/cmd_vel",
            messageType: "geometry_msgs/Twist",
        });

        const twist = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 },
        });

        cmdVel.publish(twist);
    };

    return (
        <div>
            <Joystick
                size={120}
                baseColor="#800020" // Vinotinto para el fondo
                stickColor="#A52A2A" // Vinotinto más claro para el stick
                move={handleMove}
                stop={handleStop}
                sticky={false} // Permite que regrese al centro automáticamente
            />
        </div>
    );
};

export default JoystickComponent;