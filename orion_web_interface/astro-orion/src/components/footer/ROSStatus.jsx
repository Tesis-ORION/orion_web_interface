// ROSStatus.jsx
import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

const ROSStatus = () => {
  const [online, setOnline] = useState(false);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });

    ros.on("connection", () => {
      console.log("✅ Conectado a ROS 2");
      setOnline(true);
    });

    ros.on("close", () => {
      console.log("❌ Desconectado de ROS 2");
      setOnline(false);
    });

    return () => ros.close();
  }, []);

  return (
    <div className="text-center">
      <p>
        Estado de ROS 2:{" "}
        <span style={{ color: online ? "green" : "red" }}>
          {online ? "Conectado ✅" : "Desconectado ❌"}
        </span>
      </p>
    </div>
  );
};

export default ROSStatus;
