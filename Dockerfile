# Imagen base
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias básicas
RUN apt-get update && \
    apt-get install -y \
    git \
    python3.10 \
    python3.10-venv \
    python3-pip \
    python3-dev \
    build-essential \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Establecer python3.10 como predeterminado
RUN ln -s /usr/bin/python3.10 /usr/bin/python

# Crear directorio de trabajo
WORKDIR /app

# Clonar el repositorio
RUN git clone https://github.com/unitreerobotics/xr_teleoperate.git

# Instalar dependencias de Python
WORKDIR /app/xr_teleoperate
RUN pip install --upgrade pip && \
    if [ -f requirements.txt ]; then pip install -r requirements.txt; fi

# Comando por defecto (puedes adaptarlo)
