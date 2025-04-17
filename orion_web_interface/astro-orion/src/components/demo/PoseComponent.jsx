import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Card, Container } from "react-bootstrap";

const PoseComponent = () => {
  const [pose, setPose] = useState({
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 0 },
  });

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => console.log("Conectado a ROS"));
    rosInstance.on("error", (error) => console.error("Error en ROS:", error));
    rosInstance.on("close", () => console.log("Desconectado de ROS"));

    const topic = new ROSLIB.Topic({
      ros: rosInstance,
      name: "/pose_topic",
      messageType: "geometry_msgs/Pose",
    });

    topic.subscribe((message) => {
      console.log("Recibido:", message);
      setPose(message);
    });

    return () => {
      topic.unsubscribe();
      rosInstance.close();
    };
  }, []);

  return (
    <Container className="p-3">
      <h4>Suscribirse a Pose</h4>
      <Card>
        <Card.Body>
          <Card.Text>
            <strong>Posición:</strong> <br />
            X: {pose.position.x.toFixed(2)} <br />
            Y: {pose.position.y.toFixed(2)} <br />
            Z: {pose.position.z.toFixed(2)} <br />
            <strong>Orientación:</strong> <br />
            X: {pose.orientation.x.toFixed(2)} <br />
            Y: {pose.orientation.y.toFixed(2)} <br />
            Z: {pose.orientation.z.toFixed(2)} <br />
            W: {pose.orientation.w.toFixed(2)}
          </Card.Text>
        </Card.Body>
      </Card>
    </Container>
  );
};

export default PoseComponent;
