import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

const LidarComponent = ({ rosbridgeUrl, topicName }) => {
  const canvasRef = useRef(null);
  const [scanData, setScanData] = useState(null);

  useEffect(() => {
    // Conexión a ROS
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl });

    ros.on("connection", () => console.log("Conectado a ROSBridge"));
    ros.on("error", (error) => console.error("Error en ROSBridge:", error));
    ros.on("close", () => console.log("Desconectado de ROSBridge"));

    // Suscribirse al tópico LaserScan
    const laserScanTopic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: "sensor_msgs/LaserScan",
    });

    laserScanTopic.subscribe((message) => {
      setScanData(message);
    });

    // Cleanup
    return () => {
      laserScanTopic.unsubscribe();
      ros.close();
    };
  }, [rosbridgeUrl, topicName]);

  useEffect(() => {
    if (!scanData) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");

    // Limpiar el canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const { ranges, angle_min, angle_increment } = scanData;
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const scale = 50; // Ajusta el tamaño del escaneo

    ctx.fillStyle = "red";

    ranges.forEach((range, i) => {
      if (range < scanData.range_max) {
        const angle = angle_min + i * angle_increment;
        // Invertir el eje X
        const x = centerX - range * scale * Math.cos(angle);
        const y = centerY + range * scale * Math.sin(angle);
        ctx.fillRect(x, y, 2, 2);
      }
    });
  }, [scanData]);

  return (
    <canvas
      ref={canvasRef}
      width={400}
      height={400}
      className="lidar-box"
    />
  );
};

export default LidarComponent;
