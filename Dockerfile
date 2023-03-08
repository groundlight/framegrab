FROM python:3.10-slim
ADD requirements.txt /src/
WORKDIR /src
RUN pip install -r requirements.txt
ADD ./src/ /src/
ENTRYPOINT ["python","streamlight.py"]
