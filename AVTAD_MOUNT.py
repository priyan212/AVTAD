import cv2, serial, time, threading, queue, numpy as np, torch, pygame
from ultralytics import YOLO

# CONFIG
SERIAL_PORT = "COM6"
BAUD_RATE = 115200
CAM_ID = 0
FRAME_W, FRAME_H = 640, 480
ALLOWED_NAMES = {"drone"}  # Case-insensitive match
PAN_RANGE = (10, 170)
TILT_RANGE = (20, 160)
SMOOTHING = 0.18
LOCK_LOST_FRAMES = 20
DETECT_EVERY_N = 1
HUD_COLOR = (0, 255, 0)

# Initialize Serial and pygame sound
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
time.sleep(2)

pygame.init()
pygame.mixer.init()

def play_drone_alert():
    pygame.mixer.music.load("Untitled design.mp3")  # Place in same folder
    pygame.mixer.music.play()

device = 0 if torch.cuda.is_available() else "cpu"
model = YOLO("MEDIUM.pt")  # Use your trained model path
model.fuse()
if device == 0:
    model.to(device).half()
names = model.names

cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame_q = queue.Queue(maxsize=1)
def cam_worker():
    while True:
        ok, frm = cap.read()
        if not ok:
            continue
        if frame_q.full():
            try: frame_q.get_nowait()
            except queue.Empty: pass
        frame_q.put(frm)
threading.Thread(target=cam_worker, daemon=True).start()

def px_to_angle(px, span, rng):
    return rng[0] + (rng[1]-rng[0]) * np.clip(px/span, 0, 1)

last_sent = (-999, -999)
def send_angles(pan, tilt):
    global last_sent
    p, t = int(pan), int(tilt)
    if (p, t) != last_sent:
        ser.write(f"P{p:03d} T{t:03d}\n".encode())
        last_sent = (p, t)

# Tracking state
cur_pan = np.mean(PAN_RANGE)
cur_tilt = np.mean(TILT_RANGE)
lock_id, lost_counter = None, 0
takedown_armed = False
button_pressed = False

def draw_hud(hud_layer, fps, pan, tilt, locked, sweep_phase):
    global takedown_armed
    h, w = hud_layer.shape[:2]
    cx, cy = w // 2, h // 2

    # Crosshair and radar sweep
    cv2.line(hud_layer, (cx, 0), (cx, h), (0, 255, 0), 2)
    cv2.line(hud_layer, (0, cy), (w, cy), (0, 255, 0), 2)
    angle = (sweep_phase % 360) * np.pi / 180
    x2 = int(cx + 200 * np.cos(angle))
    y2 = int(cy + 200 * np.sin(angle))
    cv2.line(hud_layer, (cx, cy), (x2, y2), (0, 255, 0), 2)

    # Panel
    panel = np.zeros((85, 200, 3), dtype=np.uint8)
    def put_outlined_text(img, text, pos, color=(0,255,0), size=0.6, thick=2):
        x, y = pos
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0,0,0), thick + 2)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thick)

    put_outlined_text(panel, f"FPS : {fps:5.1f}", (10, 25))
    put_outlined_text(panel, f"PAN : {int(pan):3d}", (10, 50))
    put_outlined_text(panel, f"TILT: {int(tilt):3d}", (10, 75))
    status_txt = "LOCKED" if locked else "SEARCH"
    status_col = (0, 0, 255) if locked else (0, 255, 255)
    put_outlined_text(panel, status_txt, (120, 50), status_col)
    hud_layer[10:95, 10:210] = panel

    # TAKEDOWN button
    btn_color = (0, 255, 0) if not takedown_armed else (0, 0, 255)
    x, y, w, h = 480, 20, 180, 45
    cv2.rectangle(hud_layer, (x, y), (x + w, y + h), btn_color, -1)
    put_outlined_text(hud_layer, "TAKEDOWN", (x + 10, y + 32), (255, 255, 255), 0.9, 2)

def mouse_callback(event, mx, my, flags, param):
    global button_pressed
    if event == cv2.EVENT_LBUTTONDOWN and takedown_armed:
        x, y, w, h = 480, 20, 180, 45
        if x <= mx <= x + w and y <= my <= y + h:
            print("TAKEDOWN CLICKED")
            button_pressed = True

cv2.namedWindow("Defence-HUD Tracker")
cv2.setMouseCallback("Defence-HUD Tracker", mouse_callback)

# Main loop
prev_t = time.time()
frame_id = 0
sweep = 0

while True:
    try:
        frame = frame_q.get(timeout=1)
    except queue.Empty:
        continue

    frame_id += 1
    run_detect = (frame_id % DETECT_EVERY_N == 0)
    target = None

    if run_detect:
        with torch.cuda.amp.autocast(enabled=device==0):
            res = model.track(frame, conf=0.4, iou=0.45, verbose=False, persist=True, device=device)[0]

        valid = []
        for b in res.boxes:
            if b.id is None: continue
            label = names[int(b.cls)]
            conf = b.conf.item()
            print(f"Detected: {label} ({conf:.2f})")  # Debug print

            if label.lower() not in ALLOWED_NAMES:
                continue

            x1, y1, x2, y2 = b.xyxy.cpu().numpy()[0]
            valid.append({
                "id": int(b.id),
                "box": (x1, y1, x2, y2),
                "area": (x2 - x1) * (y2 - y1),
                "name": label
            })

        if lock_id is not None:
            match = next((v for v in valid if v["id"] == lock_id), None)
            if match:
                lost_counter = 0
                target = match
            else:
                lost_counter += 1
                if lost_counter >= LOCK_LOST_FRAMES:
                    lock_id = None

        if lock_id is None and valid:
            target = max(valid, key=lambda v: v["area"])
            lock_id, lost_counter = target["id"], 0
            play_drone_alert()

    if target:
        x1, y1, x2, y2 = target["box"]
        cx_t, cy_t = (x1 + x2)/2, (y1 + y2)/2
        target_pan = px_to_angle(FRAME_W - cx_t, FRAME_W, PAN_RANGE)
        target_tilt = px_to_angle(FRAME_H - cy_t, FRAME_H, TILT_RANGE)
        cur_pan = cur_pan * (1 - SMOOTHING) + target_pan * SMOOTHING
        cur_tilt = cur_tilt * (1 - SMOOTHING) + target_tilt * SMOOTHING
        send_angles(cur_pan, cur_tilt)

        dx = abs(cx_t - FRAME_W / 2)
        dy = abs(cy_t - FRAME_H / 2)
        takedown_armed = dx < 80 and dy < 80

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        cv2.line(frame, (int(cx_t - 10), int(cy_t)), (int(cx_t + 10), int(cy_t)), (0, 0, 255), 1)
        cv2.line(frame, (int(cx_t), int(cy_t - 10)), (int(cx_t), int(cy_t + 10)), (0, 0, 255), 1)
        cv2.putText(frame, target["name"], (int(x1), int(y1) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        takedown_armed = False

    if button_pressed:
        print("Sending FIRE command to Arduino")
        ser.write(b"FIRE\n")
        button_pressed = False

    now = time.time()
    fps = 1.0 / (now - prev_t) if now != prev_t else 0
    prev_t = now
    sweep = (sweep + 4) % 360

    hud_overlay = np.zeros_like(frame)
    draw_hud(hud_overlay, fps, cur_pan, cur_tilt, target is not None, sweep)

    frame_inverted = cv2.bitwise_not(frame)
    final_display = cv2.addWeighted(frame_inverted, 1.0, hud_overlay, 1.0, 0)

    cv2.imshow("Defence-HUD Tracker", final_display)
    if cv2.waitKey(1) == 27:
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
