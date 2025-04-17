import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Form, Button, Container } from "react-bootstrap";

const PointPubComponent = () => {
  const [ros, setRos] = useState(null);
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [z, setZ] = useState(0);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => console.log("Conectado a ROS"));
    rosInstance.on("error", (error) => console.error("Error en ROS:", error));
    rosInstance.on("close", () => console.log("Desconectado de ROS"));

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  const publishPoint = () => {
    if (!ros) {
      console.error("No hay conexión con ROS");
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: "/point_topic",
      messageType: "geometry_msgs/Point",
    });

    const message = new ROSLIB.Message({ x: parseFloat(x), y: parseFloat(y), z: parseFloat(z) });

    topic.publish(message);
    console.log("Publicado:", message);
  };

  return (
    <Container className="p-3">
      <h4>Publicar Point</h4>
      <Form>
        <Form.Group>
          <Form.Label>X:</Form.Label>
          <Form.Control type="number" value={x} onChange={(e) => setX(e.target.value)} />
        </Form.Group>
        <Form.Group>
          <Form.Label>Y:</Form.Label>
          <Form.Control type="number" value={y} onChange={(e) => setY(e.target.value)} />
        </Form.Group>
        <Form.Group>
          <Form.Label>Z:</Form.Label>
          <Form.Control type="number" value={z} onChange={(e) => setZ(e.target.value)} />
        </Form.Group>
        <Button onClick={publishPoint} className="mt-3">
          Publicar
        </Button>
      </Form>
    </Container>
  );
};

export default PointPubComponent;
