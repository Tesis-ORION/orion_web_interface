import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

const SpeechComponent = () => {
  const [speechText, setSpeechText] = useState("Esperando datos...");

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

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

  return <div className="border p-3">{speechText}</div>;
};

export default SpeechComponent;
