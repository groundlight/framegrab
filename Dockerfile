FROM ubuntu:focal
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python-is-python3
ADD requirements.txt /src/
WORKDIR /src
# authenticate to aws codeartifact - remove this once the groundlight sdk is public
ARG AWS_SECRET_ACCESS_KEY
ARG AWS_ACCESS_KEY_ID
RUN pip install awscli
RUN aws codeartifact login --region us-west-2 --domain positronix --repository internal --tool pip
RUN pip install -r requirements.txt
ADD . /src/
ENTRYPOINT ["python","streamlight.py"]
