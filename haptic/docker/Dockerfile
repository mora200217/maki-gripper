# Dockerfile para desarrollar y compilar apps nativas para Meta Quest
FROM ubuntu:22.04

# Variables de entorno
ENV ANDROID_HOME=/opt/android-sdk
ENV ANDROID_NDK_HOME=$ANDROID_HOME/ndk/27.0.11718014
ENV PATH=$PATH:$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/platform-tools

# Instalar dependencias
RUN apt-get update && apt-get install -y \
    wget unzip openjdk-17-jdk cmake git python3 build-essential \
    && rm -rf /var/lib/apt/lists/*

# Instalar Android SDK Command-line tools
RUN mkdir -p $ANDROID_HOME/cmdline-tools
RUN wget -q https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip -O /tmp/cmdtools.zip && \
    unzip /tmp/cmdtools.zip -d $ANDROID_HOME/cmdline-tools && \
    mv $ANDROID_HOME/cmdline-tools/cmdline-tools $ANDROID_HOME/cmdline-tools/latest

# Aceptar licencias e instalar SDKs básicos
RUN yes | sdkmanager --licenses
RUN sdkmanager \
    "platform-tools" \
    "build-tools;34.0.0" \
    "platforms;android-34" \
    "ndk;27.0.11718014"

# Crear directorio de trabajo
WORKDIR /app

# Copiar tu código fuente
COPY . /app

# Comando por defecto: abre una shell lista para compilar
CMD ["/bin/bash"]

