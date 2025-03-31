import React, { useState, useEffect, useRef } from "react";
import ROSLIB from "roslib";
import { Form, Button } from "react-bootstrap";
import * as Vosk from 'vosk-browser';

const StringPubComponent = () => {
  // Estados para ROS y mensaje a publicar
  const [ros, setRos] = useState(null);
  const [finalText, setFinalText] = useState("");
  const [partialText, setPartialText] = useState("");
  const [topic, setTopic] = useState(null);
  
  // Estado para indicar si el modelo ya fue cargado
  const [modelLoaded, setModelLoaded] = useState(false);
  
  // Referencias para elementos de Vosk
  const voskModelRef = useRef(null);
  const recognizerRef = useRef(null);
  const audioContextRef = useRef(null);
  const processorRef = useRef(null);
  const mediaStreamRef = useRef(null);

  // Conexión a ROS
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: import.meta.env.VITE_ROS_URL });
    
    rosInstance.on("connection", () => {
      console.log("Conectado a ROS");
      setRos(rosInstance);
      
      // Crear el topic una vez conectado
      const topicInstance = new ROSLIB.Topic({
        ros: rosInstance,
        name: "/speech",
        messageType: "std_msgs/String",
      });
      setTopic(topicInstance);
    });

    rosInstance.on("error", (error) =>
      console.error("Error en la conexión ROS", error)
    );
    rosInstance.on("close", () => console.log("Conexión cerrada"));

    return () => rosInstance.close();
  }, []);

  // Función para cargar el modelo desde public
  const handleLoadModel = async () => {
    try {
      // Carga el modelo; la ruta es relativa a la carpeta public
      const model = await Vosk.createModel("vosk-model-small-es-0.42.zip");
      voskModelRef.current = model;
      setModelLoaded(true);
      console.log("Modelo cargado correctamente");
      
      // Crea el reconocedor a partir del modelo cargado
      recognizerRef.current = new model.KaldiRecognizer(48000);
      // Configura el evento para resultados finales
      recognizerRef.current.on("result", (message) => {
        if (message.result.text && message.result.text.trim() !== "") {
          setFinalText(prev => prev + " " + message.result.text);
          setPartialText("");
        }
      });
      
      
      // (Opcional) Si deseas mostrar resultados parciales:
      recognizerRef.current.on("partialresult", (message) => {
        setPartialText(message.result.partial);
      });
      
    } catch (error) {
      console.error("Error al cargar el modelo:", error);
    }
  };

  // Función para iniciar la escucha (streaming de audio)
  const handleStartListening = async () => {
    setFinalText("");
    setPartialText("");
    // Reinicializar el reconocedor para una nueva frase
    if (voskModelRef.current) {
      recognizerRef.current = new voskModelRef.current.KaldiRecognizer(48000);
      recognizerRef.current.on("result", (message) => {
        if (message.result.text && message.result.text.trim() !== "") {
          setFinalText(prev => prev + " " + message.result.text);
          setPartialText("");
        }
      });
      recognizerRef.current.on("partialresult", (message) => {
        setPartialText(message.result.partial);
      });
    }

    if (!modelLoaded) {
      alert("Primero debes cargar el modelo.");
      return;
    }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        audio: {
          echoCancellation: true,
          noiseSuppression: true,
          channelCount: 1,
          sampleRate: 16000
        },
      });
      mediaStreamRef.current = stream;
      
      const audioContext = new AudioContext();
      audioContextRef.current = audioContext;
      
      // Agregar el módulo worklet
      await audioContext.audioWorklet.addModule('/vosk-processor.js');
      
      // Crear el nodo worklet
      const workletNode = new AudioWorkletNode(audioContext, 'vosk-processor');
      
      // Recibir los datos de audio del worklet
      workletNode.port.onmessage = (event) => {
        const inputData = event.data; // Float32Array
        // Crear un AudioBuffer a partir de inputData (1 canal)
        const buffer = audioContext.createBuffer(1, inputData.length, audioContext.sampleRate);
        buffer.copyToChannel(inputData, 0);
        try {
          recognizerRef.current.acceptWaveform(buffer);
        } catch (error) {
          console.error("Error al procesar audio:", error);
        }
      };
      
      const source = audioContext.createMediaStreamSource(stream);
      source.connect(workletNode);
      workletNode.connect(audioContext.destination); // Opcional
      console.log("Escucha iniciada con AudioWorkletNode");
    } catch (error) {
      console.error("Error al acceder al micrófono:", error);
    }
  };
  

  // Función para detener la escucha
  const handleStopListening = () => {
    if (processorRef.current) {
      processorRef.current.disconnect();
    }
    if (audioContextRef.current) {
      audioContextRef.current.close();
    }
    if (mediaStreamRef.current) {
      mediaStreamRef.current.getTracks().forEach((track) => track.stop());
    }
    
    // Obtener y acumular el resultado final (si está disponible)
    if (recognizerRef.current.FinalResult) {
      const finalResult = recognizerRef.current.FinalResult();
      if (finalResult && finalResult.result && finalResult.result.text) {
        setFinalText(prev => prev + " " + finalResult.result.text);
        setPartialText("");
      }
    }
    console.log("Escucha detenida");
  };
  

  // Función para publicar el mensaje a ROS
  const handlePublish = () => {
    if (ros && topic) {
      const message = new ROSLIB.Message({ data: text });
      topic.publish(message);
      console.log("Mensaje publicado:", text);
    }
  };

  return (
    <div className="container mt-4">
      <h3>Publicar un String en ROS</h3>
      
      {/* Botón para cargar el modelo */}
      <Button variant="secondary" onClick={handleLoadModel} className="mb-2">
        {modelLoaded ? "Modelo cargado" : "Cargar Modelo"}
      </Button>
      
      {/* Sección de reconocimiento de voz */}
      <div className="d-flex align-items-center mb-3">
        {/* Campo de texto que muestra el resultado en streaming */}
        <Form.Control
          type="text"
          placeholder="Aquí se mostrará tu voz..."
          value={finalText + " " + partialText}
          onChange={(e) => {
            // Si permites editar manualmente, actualiza finalText y limpia partialText.
            setFinalText(e.target.value);
            setPartialText("");
          }}
        />

        {/* Botón de micrófono */}
        <Button
          variant="info"
          onMouseDown={handleStartListening}
          onMouseUp={handleStopListening}
          className="ms-2"
        >
          🎤
        </Button>
      </div>
      
      {/* Botón para publicar */}
      <Button variant="primary" onClick={handlePublish}>
        Publicar
      </Button>
    </div>
  );
};

export default StringPubComponent;