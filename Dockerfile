FROM ubuntu:focal
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python-is-python3
ADD requirements.txt /src/
WORKDIR /src
RUN pip install -r requirements.txt
ADD . /src/
ENTRYPOINT ["python","streamlight.py"]
