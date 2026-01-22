FROM python:3.11-slim

# Set environment variables
# Prevent Python from writing .pyc files to disk
ENV PYTHONDONTWRITEBYTECODE=1
# Ensure Python output is sent directly to terminal without buffering
ENV PYTHONUNBUFFERED=1
# Disable pip cache to reduce image size
ENV PIP_NO_CACHE_DIR=1
# Disable pip version check
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

# Set the working directory
WORKDIR /app

# Install system dependencies
# build-essential is often needed for compiling some Python packages
# curl is useful for healthchecks
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy only requirements first to leverage Docker layer caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --upgrade pip && \
    pip install -r requirements.txt && \
    pip install fastapi uvicorn python-multipart aiofiles

# Copy the rest of the application code
# .dockerignore should be used to exclude unnecessary files
COPY . .

# Install the current package
RUN pip install .

# Create a non-root user for security and switch to it
RUN useradd -m appuser && chown -R appuser:appuser /app
USER appuser

# Expose the port the application runs on
EXPOSE 8080

# Command to run the application using uvicorn
CMD ["uvicorn", "api.api:api", "--host", "0.0.0.0", "--port", "8080"]
