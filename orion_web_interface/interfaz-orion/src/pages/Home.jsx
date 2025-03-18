import React from "react";
import "../styles/styles.css";
import SpeechComponent from "../components/SpeechComponent";
import ImageComponent from "../components/ImageComponent";
import TwistSubComponent from "../components/TwistSubComponent";
import JoystickComponent from "../components/JoystickComponent";

const Home = () => {
    return (
      <div>
        <SpeechComponent />
        <div className="image-container">
          <ImageComponent title="Imagen a color" topic="/apc/left/image_base64" />
          <ImageComponent title="Imagen de Profundidad" topic="/apc/depth/image_base64" />
        </div>
      </div>
    );
};
  


export default Home;