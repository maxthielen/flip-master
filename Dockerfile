# building image
FROM ubuntu:20.04 as builder
MAINTAINER Max Thielen "497441@student.saxion.nl"

# Install necessary packages
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get install -y python3 python3.8-venv

# Setup the python venv
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
# Copy the pip requirements
COPY requirements.txt .
# Install python requirements
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --upgrade -r requirements.txt

# running image
FROM ubuntu:20.04

#RUN addgroup --system app && adduser --system --group app

# Install Open3D system dependencies and opencv
RUN apt-get update && apt-get install --no-install-recommends -y \
    libgl1 \
    libgomp1 \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Copy venv from builder stage
COPY --from=builder /opt/venv /opt/venv

# Copy everything to the docker container
COPY . ./

# Make data folders for python program
# RUN mkdir -p data/classifier_output data/dstv_output/products data/preprocessing_output data/product_database

# Set venv enviroment variable
ENV PATH="/opt/venv/bin:$PATH"

# Command that is run when docker run is called.
ENTRYPOINT ["python3", "src/main.py"]
