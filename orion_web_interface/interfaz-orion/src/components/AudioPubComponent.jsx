import React, { useState, useRef } from 'react';
import { Button } from 'react-bootstrap';
import { FaMicrophone } from 'react-icons/fa';
import '../styles/AudioPubComponent.css'; 

const AudioPubComponent = () => {
  const [recording, setRecording] = useState(false);
  const mediaRecorderRef = useRef(null);
  const streamRef = useRef(null);
  const audioChunksRef = useRef([]);

  const startRecording = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      streamRef.current = stream;
      mediaRecorderRef.current = new MediaRecorder(stream);
      mediaRecorderRef.current.ondataavailable = event => {
        if (event.data.size > 0) {
          audioChunksRef.current.push(event.data);
        }
      };
      mediaRecorderRef.current.start();
      setRecording(true);
    } catch (err) {
      console.error('Error al acceder al micrófono:', err);
    }
  };

  const stopRecording = () => {
    if (mediaRecorderRef.current) {
      mediaRecorderRef.current.onstop = async () => {
        const audioBlob = new Blob(audioChunksRef.current, { type: 'audio/webm' });
        audioChunksRef.current = [];
        const formData = new FormData();
        formData.append('file', audioBlob, 'grabacion.webm');

        try {
          const response = await fetch(`${import.meta.env.VITE_ORION_STT}`, {
            method: 'POST',
            body: formData,
          });
          const data = await response.json();
          console.log('Respuesta del backend:', data);
        } catch (error) {
          console.error('Error al enviar audio:', error);
        }
      };
      mediaRecorderRef.current.stop();
      setRecording(false);
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(track => track.stop());
        streamRef.current = null;
      }
    }
  };

  return (
    <div className="audio-container">
      <Button
        className={`record-button ${recording ? 'recording' : ''}`}
        onMouseDown={startRecording}
        onMouseUp={stopRecording}
        onTouchStart={startRecording}
        onTouchEnd={stopRecording}
      >
        <FaMicrophone className="microphone-icon" />
      </Button>
      <div className="button-text">
        {recording ? 'Grabando...' : 'Presiona y mantén para grabar'}
      </div>
    </div>
  );
};

export default AudioPubComponent;
