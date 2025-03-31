class VoskProcessor extends AudioWorkletProcessor {
    process(inputs, outputs, parameters) {
      // Usamos el primer canal del primer input
      const input = inputs[0];
      if (input.length > 0) {
        // Envía los datos (Float32Array) al hilo principal
        this.port.postMessage(input[0]);
      }
      return true;
    }
  }
  registerProcessor('vosk-processor', VoskProcessor);
  