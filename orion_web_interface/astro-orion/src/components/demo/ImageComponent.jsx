import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";
import "../../styles/demo.css"; 

const ImageComponent = ({ title, topic }) => {
  const [imageSrc, setImageSrc] = useState("");

  useEffect(() => {
    if (!topic) {
      console.error("❌ Error: topic es undefined o vacío");
      return;
    }

    const ros = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });
    const imageListener = new ROSLIB.Topic({
      ros,
      name: topic,
      messageType: "std_msgs/String",
    });

    imageListener.subscribe((message) => {
      if (!message.data) {
        console.error("⚠️ Mensaje de imagen vacío");
        return;
      }
      console.log("📸 Imagen Base64 recibida");
      setImageSrc(`data:image/jpeg;base64,${message.data}`);
    });

    return () => imageListener.unsubscribe();
  }, [topic]);

  return (
    <div className="image-component flex flex-col h-full w-full">
      {/* Título fuera del recuadro */}
      <div className="image-component-title text-center">
        {title}
      </div>
      <div className="image-box">
        {imageSrc ? (
          <img
            src={imageSrc}
            alt="ROS Stream"
            style={{ maxWidth: "100%", maxHeight: "100%" }}
          />
        ) : (
          <p>Cargando imagen...</p>
        )}
      </div>
    </div>
  );
};

export default ImageComponent;
