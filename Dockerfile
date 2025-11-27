# Imagen base
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Dependencias del sistema
RUN apt-get update && \
    apt-get install -y \
    git \
    wget \
    curl \
    openssl \
    build-essential \
    python3.10 \
    python3.10-venv \
    python3-pip \
    python3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Establecer Python 3.10 como predeterminado
RUN ln -sf /usr/bin/python3.10 /usr/bin/python

# Crear entorno virtual
WORKDIR /opt
RUN python -m venv /opt/tv
ENV PATH="/opt/tv/bin:$PATH"

# Instalar dependencias base (Pinocchio, numpy)
RUN pip install --upgrade pip && \
    pip install pinocchio==3.1.0 numpy==1.26.4

# Clonar el repositorio principal
WORKDIR /app
RUN git clone https://github.com/unitreerobotics/xr_teleoperate.git
WORKDIR /app/xr_teleoperate

# Inicializar submódulos de forma superficial
RUN git submodule update --init --depth 1

# Instalar televuer submódulo
WORKDIR /app/xr_teleoperate/teleop/televuer
RUN pip install -e .

# Generar certificados autofirmados para televuer
RUN openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -subj "/C=US/ST=CA/L=SF/O=Unitree/OU=AI/CN=localhost" \
    -keyout key.pem -out cert.pem

# Instalar dex-retargeting submódulo
WORKDIR /app/xr_teleoperate/teleop/robot_control/dex-retargeting
RUN pip install -e .

# Instalar dependencias del repositorio principal
WORKDIR /app/xr_teleoperate
RUN pip install -r requirements.txt

# Directorio de trabajo final
WORKDIR /app/xr_teleoperate

# Comando por defecto (puedes cambiarlo según tu uso)
CMD ["/bin/bash"]
