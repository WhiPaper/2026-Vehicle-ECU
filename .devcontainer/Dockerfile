ARG IDF_IMAGE=espressif/idf:v6.0.2
FROM ${IDF_IMAGE}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
ENV VIRTUAL_ENV=/opt/devcontainer-venv
ENV PATH="/opt/devcontainer-venv/bin:${PATH}"

ARG UDEV_VERSION=255.4-1ubuntu8.16
RUN apt-get update \
    && apt-get install -y --no-install-recommends "udev=${UDEV_VERSION}" python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-dev.txt /tmp/vehicle-ecu-requirements-dev.txt
RUN python3 -m venv "$VIRTUAL_ENV" \
    && "$VIRTUAL_ENV/bin/python" -m pip install --no-cache-dir --upgrade pip \
    && "$VIRTUAL_ENV/bin/pip" install --no-cache-dir \
        -r /tmp/vehicle-ecu-requirements-dev.txt

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]
