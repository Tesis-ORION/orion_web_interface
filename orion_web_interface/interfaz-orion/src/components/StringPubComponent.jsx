import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { Form, Button } from "react-bootstrap";
import Vosk from "vosk-browser"; // Importa la versión 0.0.8 de vosk-browser

const StringPubComponent = () => {
  const [ros, setRos] = useState(null);
  const [text, setText] = useState("");
  const [topic, setTopic] = useState(null);
  const [recognizer, setRecognizer] = useState(null);
  const [modelLoaded, setModelLoaded] = useState(false);

  useEffect(() => {
    // Conectar a ROS
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    rosInstance.on("connection", () => {
      console.log("Conectado a ROS");
      setRos(rosInstance);

      // Crear el topic una vez conectado
      const topicInstance = new ROSLIB.Topic({
        ros: rosInstance,
        name: "/speech",
        messageType: "std_msgs/String",
      });
      setTopic(topicInstance);
    });

    rosInstance.on("error", (error) =>
      console.error("Error en la conexión ROS", error)
    );
    rosInstance.on("close", () => console.log("Conexión cerrada"));

    return () => rosInstance.close();
  }, []);

  const handleLoadModel = async () => {
    try {
      // Cargar el modelo de Vosk desde la carpeta public (asegúrate de que la carpeta "model" esté allí)
      const model = await Vosk.createModel("/model");
      // Instanciar el recognizer con el modelo y la tasa de muestreo deseada (ej. 16000 Hz)
      const rec = new Vosk.Recognizer({ model: model, sampleRate: 16000 });
      setRecognizer(rec);
      setModelLoaded(true);
      console.log("Modelo Vosk y recognizer inicializados");
    } catch (error) {
      console.error("Error al cargar el modelo Vosk:", error);
    }
  };

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
        <div className="mt-3">
          <Button 
            variant="secondary" 
            onClick={handleLoadModel} 
            disabled={modelLoaded}
          >
            {modelLoaded ? "Modelo Cargado" : "Cargar Modelo Vosk"}
          </Button>
        </div>
        <Button variant="primary" className="mt-2" onClick={handlePublish}>
          Publicar
        </Button>
      </Form>
    </div>
  );
};

export default StringPubComponent;
