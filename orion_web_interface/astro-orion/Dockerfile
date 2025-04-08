FROM node:23-alpine

# Define el directorio de trabajo
WORKDIR /app

# Copia los archivos de dependencias
COPY package*.json ./

# Instala las dependencias (agrega --legacy-peer-deps si lo requieres)
RUN npm install --legacy-peer-deps

# Copia el resto de los archivos de la aplicación
COPY . .

# Construye el proyecto Astro
RUN npm run build

EXPOSE 4321


CMD ["npm", "run", "dev"]
