import asyncio
import json
import os
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Optional

from PIL import Image, ImageDraw, ImageFont


@dataclass
class AuthToken:
    token: str
    username: str
    issued_at: float
    expires_at: float


class AuthManager:
    def __init__(self):
        self._tokens: Dict[str, AuthToken] = {}

    def auth_enabled(self) -> bool:
        return os.getenv("TELEOP_AUTH_DISABLED", "false").lower() != "true"

    def validate_login(self, username: str, password: str) -> bool:
        default_user = os.getenv("TELEOP_DEFAULT_USER", "operator")
        default_pass = os.getenv("TELEOP_DEFAULT_PASSWORD", "operator")
        return username == default_user and password == default_pass

    def issue_token(self, username: str, ttl_seconds: int = 12 * 60 * 60) -> str:
        now = time.time()
        token = uuid.uuid4().hex
        self._tokens[token] = AuthToken(
            token=token,
            username=username,
            issued_at=now,
            expires_at=now + ttl_seconds,
        )
        return token

    def verify_token(self, token: Optional[str]) -> Optional[AuthToken]:
        if not token:
            return None
        t = self._tokens.get(token)
        if not t:
            return None
        if time.time() > t.expires_at:
            self._tokens.pop(token, None)
            return None
        return t

    def revoke(self, token: str) -> None:
        self._tokens.pop(token, None)


class SessionRecorder:
    def __init__(self, recordings_dir: Optional[str] = None):
        base_dir = recordings_dir or os.getenv("TELEOP_RECORDINGS_DIR")
        if not base_dir:
            repo_dir = os.path.dirname(os.path.dirname(__file__))
            base_dir = os.path.join(repo_dir, "server", "recordings")
        self._dir = base_dir
        self._sessions: Dict[str, Any] = {}
        self._session_dirs: Dict[str, str] = {}
        self._last_frame_ts: Dict[str, float] = {}
        os.makedirs(self._dir, exist_ok=True)

    def start(self, username: str) -> str:
        session_id = uuid.uuid4().hex
        session_dir = os.path.join(self._dir, f"session_{session_id}")
        frames_dir = os.path.join(session_dir, "frames")
        os.makedirs(frames_dir, exist_ok=True)
        path = os.path.join(session_dir, "events.jsonl")
        f = open(path, "a", encoding="utf-8")
        self._sessions[session_id] = f
        self._session_dirs[session_id] = frames_dir
        self._last_frame_ts[session_id] = 0.0
        self.write(session_id, {"type": "session_start", "ts": time.time(), "username": username})
        return session_id

    def write(self, session_id: str, event: Dict[str, Any]) -> None:
        f = self._sessions.get(session_id)
        if not f:
            return
        f.write(json.dumps(event, ensure_ascii=False) + "\n")
        f.flush()

    def stop(self, session_id: str) -> None:
        f = self._sessions.pop(session_id, None)
        self._session_dirs.pop(session_id, None)
        self._last_frame_ts.pop(session_id, None)
        if not f:
            return
        try:
            f.write(json.dumps({"type": "session_end", "ts": time.time()}, ensure_ascii=False) + "\n")
            f.flush()
        finally:
            f.close()

    def save_frame(self, session_id: str, jpg: bytes, min_interval_s: float = 1.0) -> None:
        frames_dir = self._session_dirs.get(session_id)
        if not frames_dir:
            return
        now = time.time()
        last = self._last_frame_ts.get(session_id, 0.0)
        if now - last < min_interval_s:
            return
        self._last_frame_ts[session_id] = now
        name = f"frame_{int(now * 1000)}.jpg"
        path = os.path.join(frames_dir, name)
        with open(path, "wb") as f:
            f.write(jpg)


def render_status_frame(width: int, height: int, state: Dict[str, Any]) -> bytes:
    img = Image.new("RGB", (width, height), (12, 18, 32))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default()
    except Exception:
        font = None

    draw.rectangle([16, 16, width - 16, height - 16], outline=(40, 64, 110), width=2)
    draw.line([width // 2, 24, width // 2, height - 24], fill=(25, 40, 70), width=1)
    draw.line([24, height // 2, width - 24, height // 2], fill=(25, 40, 70), width=1)
    draw.line([24, height - 36, 90, height - 36], fill=(90, 140, 255), width=3)
    draw.line([24, height - 36, 24, height - 102], fill=(110, 255, 150), width=3)

    pos = state.get("position") or [0.0, 0.0, 0.0]
    ori = state.get("orientation") or [1.0, 0.0, 0.0, 0.0]
    status = state.get("status") or "unknown"
    ts = state.get("timestamp") or 0.0

    lines = [
        f"Teleop Video (Mock) | status={status}",
        f"pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]",
        f"quat: [{ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f}, {ori[3]:.3f}]",
        f"ts: {ts:.3f}",
    ]

    y = 28
    for line in lines:
        draw.text((28, y), line, fill=(230, 240, 255), font=font)
        y += 18

    px = int(width // 2 + float(pos[0]) * 140)
    py = int(height // 2 - float(pos[1]) * 140)
    r = 10
    draw.ellipse([px - r, py - r, px + r, py + r], fill=(255, 210, 90), outline=(20, 20, 20))

    from io import BytesIO

    buf = BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return buf.getvalue()


async def mjpeg_stream(get_state_fn=None, fps: int = 10, get_frame_fn=None):
    delay = 1.0 / max(1, fps)
    boundary = b"frame"
    while True:
        jpg = None
        if get_frame_fn:
            try:
                jpg = get_frame_fn()
            except Exception:
                pass
        
        if jpg is None and get_state_fn:
            state = get_state_fn()
            jpg = render_status_frame(960, 540, state)
            
        if jpg:
            headers = (
                b"--" + boundary + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8")
            )
            yield headers + jpg + b"\r\n"
        await asyncio.sleep(delay)
