"""Live MJPEG aiming preview, served from the Pi. Bench tool.

The Pi runs headless with opencv-python-headless, so there is no window to open
there. This serves the camera as an MJPEG stream that any browser renders, with
focus/exposure numbers burned into the image so aiming is a closed loop instead
of a guess.

Loopback only, same as the arm bridge: reach it through the SSH tunnel.
"""

import http.server
import socketserver
import threading

import cv2

HOST, PORT, INDEX = "127.0.0.1", 8781, 0

cap = cv2.VideoCapture(INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    raise SystemExit("camera would not open")
lock = threading.Lock()


def annotated():
    with lock:
        frame = None
        for _ in range(2):
            ok, frame = cap.read()
    if frame is None:
        return None
    # The camera is mounted upside down, so every frame arrives rotated 180.
    # Rotate BEFORE the overlay is drawn, or the focus/light banner and the
    # crosshair come out upside down too. Doing it here also means /snapshot,
    # /stream and every measurement script see the same upright picture - one
    # place, not three.
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    bright = float(gray.mean())
    dark = float((gray < 40).mean() * 100)
    h, w = frame.shape[:2]
    for i in (1, 2):
        cv2.line(frame, (w * i // 3, 0), (w * i // 3, h), (60, 60, 60), 1)
        cv2.line(frame, (0, h * i // 3), (w, h * i // 3), (60, 60, 60), 1)
    cv2.drawMarker(frame, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 40, 1)
    ok_focus, ok_light = sharp >= 100, (60 <= bright <= 190 and dark < 35)
    colour = (0, 220, 0) if (ok_focus and ok_light) else (0, 165, 255)
    cv2.rectangle(frame, (0, 0), (w, 76), (0, 0, 0), -1)
    cv2.putText(frame, f"FOCUS {sharp:7.1f}  {'OK' if ok_focus else 'BLURRY'}",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    cv2.putText(frame, f"LIGHT {bright:5.1f}/255  dark {dark:4.1f}%  {'OK' if ok_light else 'TOO DARK'}",
                (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return jpg.tobytes() if ok else None


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/snapshot"):
            data = annotated()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    data = annotated()
                    if data is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
        html = (b"<html><head><title>ARM CAMERA AIM</title>"
                b"<style>body{background:#111;color:#ddd;font-family:sans-serif;margin:0;padding:12px}"
                b"img{max-width:100%;border:1px solid #333}</style></head><body>"
                b"<h3>Arm camera - live. Green numbers = good enough for motion evidence.</h3>"
                b"<img src='/stream'></body></html>")
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


print(f"live preview on http://{HOST}:{PORT}/", flush=True)
Server((HOST, PORT), Handler).serve_forever()
