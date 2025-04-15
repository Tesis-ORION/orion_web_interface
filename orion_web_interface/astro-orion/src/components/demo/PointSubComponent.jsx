import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Card, Container } from "react-bootstrap";

const PointSubComponent = () => {
  const [point, setPoint] = useState({ x: 0, y: 0, z: 0 });

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => console.log("Conectado a ROS"));
    rosInstance.on("error", (error) => console.error("Error en ROS:", error));
    rosInstance.on("close", () => console.log("Desconectado de ROS"));

    const topic = new ROSLIB.Topic({
      ros: rosInstance,
      name: "/point_topic",
      messageType: "geometry_msgs/Point",
    });

    topic.subscribe((message) => {
      console.log("Recibido:", message);
      setPoint(message);
    });

    return () => {
      topic.unsubscribe();
      rosInstance.close();
    };
  }, []);

  return (
    <Container className="p-3">
      <h4>Suscribirse a Point</h4>
      <Card>
        <Card.Body>
          <Card.Text>
            <strong>X:</strong> {point.x.toFixed(2)} <br />
            <strong>Y:</strong> {point.y.toFixed(2)} <br />
            <strong>Z:</strong> {point.z.toFixed(2)}
          </Card.Text>
        </Card.Body>
      </Card>
    </Container>
  );
};

export default PointSubComponent;