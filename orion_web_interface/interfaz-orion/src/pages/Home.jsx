import React from "react";
import "../styles/styles.css";
import SpeechComponent from "../components/SpeechComponent";
import ImageComponent from "../components/ImageComponent";
import TwistSubComponent from "../components/TwistSubComponent";
import JoystickComponent from "../components/JoystickComponent";
import LidarComponent from "../components/LidarComponent";
import AudioPubComponent from "../components/AudioPubComponent";

const Home = () => {
    return (
      <div>
        <SpeechComponent />
        <AudioPubComponent />
        <LidarComponent rosbridgeUrl="ws://localhost:9090" topicName="/scan"/>
        <div className="image-container">
          <ImageComponent title="Imagen a color" topic="/apc/left/image_base64" />
          <ImageComponent title="Imagen de Profundidad" topic="/apc/depth/image_base64" />
        </div>
      </div>
    );
};
  


export default Home;