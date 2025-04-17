import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Card, Container } from "react-bootstrap";

const TwistSubComponent = () => {
  const [twist, setTwist] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  });

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => console.log("Conectado a ROS"));
    rosInstance.on("error", (error) => console.error("Error en ROS:", error));
    rosInstance.on("close", () => console.log("Desconectado de ROS"));

    const topic = new ROSLIB.Topic({
      ros: rosInstance,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });

    topic.subscribe((message) => {
      console.log("Recibido:", message);
      setTwist(message);
    });

    return () => {
      topic.unsubscribe();
      rosInstance.close();
    };
  }, []);

  return (
    <Container className="p-3">
      <h4>Suscribirse a Twist</h4>
      <Card>
        <Card.Body>
          <Card.Text>
            <strong>Velocidad Lineal:</strong> <br />
            X: {twist.linear.x.toFixed(2)} <br />
            Y: {twist.linear.y.toFixed(2)} <br />
            Z: {twist.linear.z.toFixed(2)} <br />
            <strong>Velocidad Angular:</strong> <br />
            X: {twist.angular.x.toFixed(2)} <br />
            Y: {twist.angular.y.toFixed(2)} <br />
            Z: {twist.angular.z.toFixed(2)}
          </Card.Text>
        </Card.Body>
      </Card>
    </Container>
  );
};

export default TwistSubComponent;
