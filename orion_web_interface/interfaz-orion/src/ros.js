import ROSLIB from "roslib";

// Conectar a ROSBridge
const ros = new ROSLIB.Ros({
    url: "ws://localhost:9090"  // Asegúrate de que este puerto es el correcto
});

// Manejo de errores y conexión
ros.on("connection", () => console.log("Conectado a ROS!"));
ros.on("error", (error) => console.error("Error en ROS:", error));
ros.on("close", () => console.warn("Conexión con ROS cerrada"));

// Función para convertir ArrayBuffer a Base64
const arrayBufferToBase64 = (buffer) => {
    let binary = '';
    let bytes = new Uint8Array(buffer);
    let len = bytes.byteLength;
    for (let i = 0; i < len; i++) {
        binary += String.fromCharCode(bytes[i]);
    }
    return btoa(binary);
};


// Función para suscribirse a un tópico
const subscribeToTopic = (topicName, messageType, callback) => {
    if (!topicName) {
        console.error("Error: topicName es undefined o vacío");
        return null;
    }

    console.log(`📡 Suscribiéndose a ${topicName} con tipo ${messageType}`);

    const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });

    topic.subscribe((message) => {
        callback(message);
    });

    return topic;
};


// Función para publicar en un tópico
const publishToTopic = (topicName, messageType, message) => {
    const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType
    });

    const msg = new ROSLIB.Message(message);
    topic.publish(msg);
};

// Exportar funciones
export { ros, subscribeToTopic, publishToTopic, arrayBufferToBase64 };
