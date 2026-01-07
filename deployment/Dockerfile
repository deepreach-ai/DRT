FROM python:3.11-slim
WORKDIR /app/server
COPY server/requirements.txt /app/server/requirements.txt
RUN pip install --no-cache-dir -r /app/server/requirements.txt
COPY server /app/server
ENV TELEOP_BACKEND=mock
EXPOSE 8000
EXPOSE 9000
CMD ["uvicorn", "teleop_server:app", "--host", "0.0.0.0", "--port", "8000"]
