import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Form, Button } from "react-bootstrap";

const StringPubComponent = () => {
  const [ros, setRos] = useState(null);
  const [text, setText] = useState("");
  const [topic, setTopic] = useState(null);

  useEffect(() => {
    // Conectar a ROS
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => {
      console.log("Conectado a ROS");
      setRos(rosInstance);
      
      // Crear el topic una vez ROS esté conectado
      const topicInstance = new ROSLIB.Topic({
        ros: rosInstance,
        name: "/speech",
        messageType: "std_msgs/String",
      });
      setTopic(topicInstance);
    });

    rosInstance.on("error", (error) => console.error("Error en la conexión ROS", error));
    rosInstance.on("close", () => console.log("Conexión cerrada"));

    return () => rosInstance.close();
  }, []);

  const handlePublish = () => {
    if (ros && topic) {
      const message = new ROSLIB.Message({ data: text });
      topic.publish(message);
      console.log("Mensaje publicado:", text);
    }
  };

  return (
    <div className="container mt-4">
      <h3>Publicar un String en ROS</h3>
      <Form>
        <Form.Group controlId="formText">
          <Form.Label>Mensaje:</Form.Label>
          <Form.Control
            type="text"
            placeholder="Escribe un mensaje"
            value={text}
            onChange={(e) => setText(e.target.value)}
          />
        </Form.Group>
        <Button variant="primary" className="mt-2" onClick={handlePublish}>
          Publicar
        </Button>
      </Form>
    </div>
  );
};

export default StringPubComponent;