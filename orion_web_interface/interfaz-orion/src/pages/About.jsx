import React from "react";
import { Container, Row, Col, Card } from "react-bootstrap";

const teamMembers = [
  {
    name: "Daniel Felipe Lopez Escobar",
    careers: ["Ingeniería Mecatrónica"],
    description: "Estudiante de Ingeniería Mecatrónica de la Pontificia Universidad Javeriana. Miembro del semillero y capítulo estudiantil IEEE RAS Javeriana. Apasionado por el diseño, la robótica y los libros.",
    img: "https://avatars.githubusercontent.com/u/79463480?v=4",
  },
  {
    name: "Miguel Angel González Rodríguez",
    careers: ["Ingeniería de Sistemas", "Ingeniería Mecatrónica"],
    description: "Estudiante con un enfoque en la inteligencia artificial aplicada a la robótica. Representante estudiantil de RAS Sección Colombia. Experiencia en Python, C++, Go, DevOps, cloud computing y contenedores, con un interés particular en la integración de IA en arquitecturas robóticas modulares y eficientes.",
    img: "https://avatars.githubusercontent.com/u/49737722?v=4",
  },
  {
    name: "Alejandro Bermudez Fajardo",
    careers: ["Ingeniería de Sistemas"],
    description: "Estudiante de ingeniería de sistemas de la Pontificia Universidad Javeriana apasionado por la robótica, la inteligencia artificial y la programación en la nube.",
    img: "https://avatars.githubusercontent.com/u/133521849?v=4",
  },
];

const About = () => {
  return (
    <Container className="mt-4">
      <h1 className="text-center mb-4">Acerca de Nosotros</h1>
      <Row className="justify-content-center">
        {teamMembers.map((member, index) => (
          <Col key={index} md={4} sm={6} xs={12} className="mb-4">
            <Card className="shadow-lg text-center">
              <Card.Img variant="top" src={member.img} className="rounded-circle p-3" />
              <Card.Body>
                <Card.Title>{member.name}</Card.Title>
                <Card.Subtitle className="mb-2 text-muted">
                  {member.careers.map((career, i) => (
                    <div key={i}>{career}</div> // Separa las carreras en líneas diferentes
                  ))}
                </Card.Subtitle>
                <Card.Text>{member.description}</Card.Text>
              </Card.Body>
            </Card>
          </Col>
        ))}
      </Row>
    </Container>
  );
};

export default About;
