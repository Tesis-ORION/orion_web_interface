import React, { useState, useRef, useEffect } from 'react';
import { FaMicrophone } from 'react-icons/fa';
import ROSLIB from 'roslib';
import '../../styles/demo.css';

const AudioPubComponent = () => {
  const [recording, setRecording] = useState(false);
  const mediaRecorderRef = useRef(null);
  const streamRef = useRef(null);
  const audioChunksRef = useRef([]);
  const rosRef = useRef(null);
  const webTopicRef = useRef(null);
  const infoTopicRef = useRef(null);
  const dataTopicRef = useRef(null);
  const audioContextRef = useRef(null);

  // Conectar a ROS y configurar topics
  useEffect(() => {
    const ros = new ROSLIB.Ros();
    rosRef.current = ros;
    ros.connect(import.meta.env.VITE_ROS_URL);

    ros.on('connection', () => {
      console.log('🔗 Conectado a ROS');

      webTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/web',
        messageType: 'std_msgs/Bool',
      });
      infoTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/audio_info',
        messageType: 'audio_messages/AudioInfo',
      });
      dataTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/audio_data',
        messageType: 'audio_messages/AudioData',
      });

      audioContextRef.current = new (window.AudioContext || window.webkitAudioContext)();
    });

    ros.on('error', (err) => {
      console.error('❌ Error de conexión ROS:', err);
    });

    ros.on('close', () => {
      console.log('🔌 Desconectado de ROS');
    });

    return () => {
      ros.close();
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(t => t.stop());
      }
    };
  }, []);

  const startRecording = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      streamRef.current = stream;
      mediaRecorderRef.current = new MediaRecorder(stream, { mimeType: 'audio/webm' });
      audioChunksRef.current = [];

      mediaRecorderRef.current.ondataavailable = (e) => {
        if (e.data.size > 0) audioChunksRef.current.push(e.data);
      };

      mediaRecorderRef.current.start();
      setRecording(true);
    } catch (err) {
      console.error('Error al acceder al micrófono:', err);
    }
  };

  const stopRecording = () => {
    if (!mediaRecorderRef.current || !audioContextRef.current) return;

    mediaRecorderRef.current.onstop = async () => {
      const blob = new Blob(audioChunksRef.current, { type: 'audio/webm' });
      streamRef.current.getTracks().forEach(t => t.stop());
      setRecording(false);

      try {
        const arrayBuffer = await blob.arrayBuffer();
        const audioBuffer = await audioContextRef.current.decodeAudioData(arrayBuffer);
        const channelData = audioBuffer.getChannelData(0);

        // 1) Publicar true en /web
        const webMsg = new ROSLIB.Message({ data: true });
        webTopicRef.current.publish(webMsg);

        // 2) Publicar AudioInfo
        const infoMsg = new ROSLIB.Message({
          num_channels: audioBuffer.numberOfChannels,
          sample_rate: audioBuffer.sampleRate,
          subtype: 'float32',
          uuid: `${Date.now()}`
        });
        infoTopicRef.current.publish(infoMsg);

        // 3) Publicar AudioData
        const dataMsg = new ROSLIB.Message({
          data: Array.from(channelData)
        });
        dataTopicRef.current.publish(dataMsg);

        console.log('✅ Audio publicado:', {
          uuid: infoMsg.uuid,
          samples: channelData.length
        });
      } catch (err) {
        console.error('Error al procesar y publicar audio:', err);
      }
    };

    mediaRecorderRef.current.stop();
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
