# skysim shared-world physics server.
#
# Built as a container so the same binary can run beside SITL locally and as a
# sidecar container inside the SITL ECS task on Fargate. An awsvpc task cannot
# reach a skysim running on the host, so it has to travel with the task and be
# addressed over 127.0.0.1 within the task's network namespace.
#
# Build:  docker build -t skyhub-skysim:local .
# Run:    docker run --rm -p 8642:8642 -p 9002-9202:9002-9202/udp skyhub-skysim:local
#
# The tile set is NOT baked in: worlds are cooked separately (tools/cooker) and
# mounted or synced to SKYSIM_TILES so the image does not have to be rebuilt for
# a new city.

# ---------------------------------------------------------------- build stage
FROM ubuntu:22.04 AS build

ENV DEBIAN_FRONTEND=noninteractive

# cmake >= 3.24 is required; 22.04 ships 3.22, so take it from Kitware's repo.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        gnupg \
        ninja-build \
        g++ \
        make \
    && curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" \
        > /etc/apt/sources.list.d/kitware.list \
    && apt-get update && apt-get install -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt ./
COPY src/ ./src/
COPY tools/ ./tools/
COPY tests/ ./tests/

# Jolt and httplib are pulled by FetchContent during configure
RUN cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && cmake --build build -j "$(nproc)" --target skysim tile_cooker

# --------------------------------------------------------------- runtime stage
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        libstdc++6 \
        python3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /src/build/skysim /usr/local/bin/skysim
COPY --from=build /src/build/tile_cooker /usr/local/bin/tile_cooker
# Cooking tools travel with the image so a world can be regenerated in place
COPY --from=build /src/tools/cooker/ /opt/skysim/cooker/

# Control plane (gateway -> skysim). 0.0.0.0 because the caller is another
# container, not the same process.
ENV SKYSIM_API_BIND=0.0.0.0 \
    SKYSIM_API_PORT=8642 \
    SKYSIM_TILES="" \
    SKYSIM_STREAM_RADIUS=1500 \
    SKYSIM_STREAM_MAX=128 \
    SKYSIM_VEHICLES=0 \
    SKYSIM_EXTRA_ARGS=""

EXPOSE 8642
# ArduPilot json physics: 9002 + 10*instance, one per vehicle
EXPOSE 9002-9202/udp

COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
