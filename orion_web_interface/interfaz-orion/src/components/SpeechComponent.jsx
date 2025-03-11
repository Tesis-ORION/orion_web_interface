import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

const SpeechComponent = () => {
  const [speechText, setSpeechText] = useState("Esperando datos...");

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    const speechTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/speech",
      messageType: "std_msgs/String",
    });

    speechTopic.subscribe((message) => {
      setSpeechText(message.data);
    });

    return () => speechTopic.unsubscribe();
  }, []);

  return (
    <div className="border p-3">
      <h5 className="text-center image-box-title">Tópico Speech</h5>
      <div className="speech-box">{speechText}</div>
    </div>
  );
};

export default SpeechComponent;
