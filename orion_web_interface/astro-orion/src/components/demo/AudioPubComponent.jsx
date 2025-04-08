import React, { useState, useRef } from 'react';
import { FaMicrophone } from 'react-icons/fa';
import '../../styles/demo.css';

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
    <div className="audio-container flex flex-col items-center gap-4">
      <button
        onMouseDown={startRecording}
        onMouseUp={stopRecording}
        onTouchStart={startRecording}
        onTouchEnd={stopRecording}
        className="audio-pub-button"
      >
        <FaMicrophone className="text-white text-2xl" />
      </button>
      <p className="audio-pub-text text-center">
        {recording ? 'Grabando...' : 'Presiona y mantén para grabar'}
      </p>
    </div>
  );
};

export default AudioPubComponent;
