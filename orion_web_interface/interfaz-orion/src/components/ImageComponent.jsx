import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

const ImageComponent = ({ title, topic }) => {
  const [imageSrc, setImageSrc] = useState("");

  useEffect(() => {
    if (!topic) {
      console.error("❌ Error: topic es undefined o vacío");
      return;
    }

    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const imageListener = new ROSLIB.Topic({
      ros,
      name: topic, // Debe ser "/apc/left/image_base64"
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
    <div className="image-box">
      <div className="image-box-title">{title}</div>
      <br />
      {imageSrc ? (
        <img src={imageSrc} alt="ROS Stream" width="320" height="240" />
      ) : (
        <p>Cargando imagen...</p>
      )}
    </div>
  );
};

export default ImageComponent;
