import React, { useEffect, useState } from "react";
import { subscribeToTopic } from "./ros";  // Importamos la conexión con ROS
import "./styles/styles.css";

const Navbar = () => <div className="navbar">ROS 2 Dashboard</div>;

const SpeechComponent = () => {
  const [text, setText] = useState("Esperando datos...");

  useEffect(() => {
    const topic = subscribeToTopic("/speech", "std_msgs/msg/String", (message) => {
      setText(message.data);
    });

    return () => topic.unsubscribe();  // Limpiar la suscripción cuando el componente se desmonte
  }, []);

  return <div className="speech-box">{text}</div>;
};

const ImageComponent = ({ title, topicName }) => {
  const [imageSrc, setImageSrc] = useState(null);

  useEffect(() => {
    const topic = subscribeToTopic(topicName, "std_msgs/msg/String", (message) => {
      if (message.data) {
        setImageSrc(`data:image/jpeg;base64,${message.data}`);
      }
    });

    return () => topic.unsubscribe();
  }, [topicName]);

  return (
    <div className="image-box">
      {title}
      <br />
      {imageSrc ? <img src={imageSrc} alt={title} width="200" /> : "Esperando imagen..."}
    </div>
  );
};

const Footer = () => <div className="footer">Footer</div>;

const App = () => {
  return (
    <div>
      <Navbar />
      <div className="container">
        <SpeechComponent />
        <div className="image-container">
          <ImageComponent title="Imagen a color" topicName="/apc/left/image_base64" />
          <ImageComponent title="Imagen de Profundidad" topicName="/apc/depth/image_base64" />
        </div>
      </div>
      <Footer />
    </div>
  );
};

export default App;
