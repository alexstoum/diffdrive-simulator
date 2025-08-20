FROM python:3.11-slim

WORKDIR /work

# System deps (optional: for matplotlib backends)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml README.md ./
COPY src ./src

RUN pip install --upgrade pip && pip install -e .

ENTRYPOINT ["diffdrive-sim"]