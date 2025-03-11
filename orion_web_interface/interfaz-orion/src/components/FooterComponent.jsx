import React, { useEffect, useState } from "react";
import { Container, Row, Col } from "react-bootstrap";
import ROSLIB from "roslib";

const FooterComponent = () => {
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

    return () => ros.close(); // Cierra la conexión al desmontar
  }, []);

  return (
    <footer className="footer">
      <Container>
        <Row className="text-center">
          <Col md={4}>
            <p>© 2025 ORION Dashboard</p>
          </Col>
          <Col md={4}>
            <p>
              Estado de ROS 2:{" "}
              <span style={{ color: online ? "green" : "red" }}>
                {online ? "Conectado ✅" : "Desconectado ❌"}
              </span>
            </p>
          </Col>
          <Col md={4}>
            <a
              href="https://github.com/orgs/Tesis-ORION/repositories"
              target="_blank"
              rel="noopener noreferrer"
            >
              GitHub
            </a>{" "}
            |{" "}
            <a
              href="https://www.linkedin.com/in/miguelgonrod/"
              target="_blank"
              rel="noopener noreferrer"
            >
              Miguel
            </a>{" "}
            |{" "}
            <a
              href="https://www.linkedin.com/in/alexoberco/"
              target="_blank"
              rel="noopener noreferrer"
            >
              Alejandro
            </a>{" "}
            |{" "}
            <a
              href="https://www.linkedin.com/in/daniel-f-lopez-e/"
              target="_blank"
              rel="noopener noreferrer"
            >
              Daniel
            </a>
          </Col>
        </Row>
      </Container>
    </footer>
  );
};

export default FooterComponent;
