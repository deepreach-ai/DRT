"""
Simple HTTP server to serve the web UI
"""
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import os

# Get the path to the web directory
current_dir = os.path.dirname(os.path.abspath(__file__))
web_dir = os.path.join(current_dir, "web")
static_dir = os.path.join(web_dir, "static")

# Create FastAPI app for serving web UI
web_app = FastAPI(title="Teleoperation Web UI")

# Mount static files
if os.path.exists(static_dir):
    web_app.mount("/static", StaticFiles(directory=static_dir), name="static")

@web_app.get("/")
async def serve_index():
    """Serve the main web interface"""
    index_path = os.path.join(web_dir, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    else:
        return {"error": "Web UI not found", "path": index_path}

if __name__ == "__main__":
    import uvicorn
    print(f"Starting Web UI server...")
    print(f"Web directory: {web_dir}")
    print(f"Open browser to: http://localhost:8080")
    uvicorn.run(web_app, host="0.0.0.0", port=8080)
