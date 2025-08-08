import cv2, serial, time, queue, threading, torch
import numpy as np
from ultralytics import YOLO

# ======= CONFIG ========
SERIAL_PORT = "COM6"
BAUD_RATE = 115200
CAM_ID = 0
FRAME_W, FRAME_H = 640, 480
ALLOWED_NAMES = {"person"}

# ======= SERIAL INIT ========
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
time.sleep(2)

# ======= YOLO INIT ========
device = 0 if torch.cuda.is_available() else "cpu"
model = YOLO("yolov8n.pt")
model.fuse()
if device == 0:
    model.to(device).half()
names = model.names

# ======= CAMERA INIT ========
cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame_q = queue.Queue(maxsize=1)
def cam_worker():
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if frame_q.full():
            try: frame_q.get_nowait()
            except queue.Empty: pass
        frame_q.put(frame)
threading.Thread(target=cam_worker, daemon=True).start()

# ======= PID PARAMETERS ========
Kp_x, Kd_x = 0.005, 0.001
Kp_y, Kd_y = 0.007, 0.001
prev_error_x = 0
prev_error_y = 0
prev_time = time.time()

lock_id, lost_counter = None, 0
last_command = "S"

# ======= MAIN LOOP ========
while True:
    try:
        frame = frame_q.get(timeout=1)
    except queue.Empty:
        continue

    now = time.time()
    frame_center_x = FRAME_W / 2
    frame_center_y = FRAME_H / 2
    target = None

    # YOLO detection
    with torch.cuda.amp.autocast(enabled=device==0):
        res = model.track(frame, conf=0.3, iou=0.45, verbose=False, persist=True, device=device)[0]

    valid = []
    for b in res.boxes:
        if b.id is None: continue
        label = names[int(b.cls)]
        if label not in ALLOWED_NAMES: continue
        x1, y1, x2, y2 = b.xyxy.cpu().numpy()[0]
        valid.append({"id": int(b.id), "box": (x1, y1, x2, y2), "area": (x2 - x1)*(y2 - y1)})

    # Lock onto same person
    if lock_id is not None:
        match = next((v for v in valid if v["id"] == lock_id), None)
        if match:
            target = match
            lost_counter = 0
        else:
            lost_counter += 1
            if lost_counter >= 10:
                lock_id = None
    elif valid:
        target = max(valid, key=lambda v: v["area"])
        lock_id, lost_counter = target["id"], 0

    # ==== TRACKING ====
    if target:
        x1, y1, x2, y2 = target["box"]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        error_x = cx - frame_center_x  # +ve = too right
        error_y = cy - frame_center_y  # +ve = too low

        dt = now - prev_time if now != prev_time else 0.01
        d_error_x = (error_x - prev_error_x) / dt
        d_error_y = (error_y - prev_error_y) / dt

        output_x = Kp_x * error_x + Kd_x * d_error_x
        output_y = Kp_y * error_y + Kd_y * d_error_y

        prev_error_x = error_x
        prev_error_y = error_y
        prev_time = now

        # Decision logic
        threshold = 50
        cmd = "S"
        if abs(error_x) > threshold:
            cmd = "RIGHT" if output_x > 0 else "LEFT"
        elif abs(error_y) > threshold:
            cmd = "BACK" if output_y > 0 else "FORWARD"

        if cmd != last_command:
            ser.write((cmd + "\n").encode())
            print(f"[CMD] {cmd}")
            last_command = cmd

        # Draw box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
        cv2.putText(frame, "person", (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

    else:
        if last_command != "S":
            ser.write(b"S\n")
            print("[CMD] S (no target)")
            last_command = "S"

    cv2.imshow("Omni-PID Tracker", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
