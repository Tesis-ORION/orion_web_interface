import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import ROSLIB from "roslib";

const PointCloudComponent = () => {
  const mountRef = useRef(null);
  const [pointCloud, setPointCloud] = useState([]);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const pointCloudTopic = new ROSLIB.Topic({
      ros,
      name: "/apc/points/data_raw",
      messageType: "sensor_msgs/PointCloud2",
    });

    pointCloudTopic.subscribe((message) => {
      const points = convertPointCloud2(message);
      setPointCloud(points);
    });

    return () => {
      pointCloudTopic.unsubscribe();
      ros.close();
    };
  }, []);

  useEffect(() => {
    if (!mountRef.current || pointCloud.length === 0) return;

    // Configuración de Three.js
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 0, 2);

    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, window.innerHeight);
    mountRef.current.appendChild(renderer.domElement);

    // Crear geometría de puntos
    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(pointCloud.flat());
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const material = new THREE.PointsMaterial({ color: 0x00ff00, size: 0.02 });
    const points = new THREE.Points(geometry, material);
    scene.add(points);

    // Animación
    const animate = () => {
      requestAnimationFrame(animate);
      points.rotation.y += 0.01;
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      mountRef.current.removeChild(renderer.domElement);
    };
  }, [pointCloud]);

  return <div ref={mountRef} style={{ width: "100%", height: "500px" }} />;
};

// 🔹 Conversión de `PointCloud2` a un array de coordenadas 3D
const convertPointCloud2 = (message) => {
  const points = [];
  const { data, width, height, point_step } = message; // Eliminamos `row_step`

  if (!data || data.length === 0) return points;

  // Convertir `data` a un `ArrayBuffer`
  const buffer = new Uint8Array(data).buffer;
  const view = new DataView(buffer);

  const totalPoints = width * height;

  for (let i = 0; i < totalPoints; i++) {
    const offset = i * point_step;

    // Validación de límites
    if (offset + 8 >= buffer.byteLength) break;

    const x = view.getFloat32(offset, true);
    const y = view.getFloat32(offset + 4, true);
    const z = view.getFloat32(offset + 8, true);

    points.push([x, y, z]);
  }

  return points;
};

export default PointCloudComponent;
