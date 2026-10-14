
import cv2
import time

def scan_qr(frame, detector):
    success, decoded_info, points, _ = detector.detectAndDecodeMulti(frame)

    if not success:
        return [], points

    results = []

    for data in decoded_info:
        if not data:
            continue

        try:
            coordinates = {}

            for part in data.split("&"):
                key, value = part.split("=")
                coordinates[key.strip().upper()] = float(value)

            x = coordinates["X"]
            y = coordinates["Y"]

            results.append((data, x, y))

        except (ValueError, KeyError):
            continue

    return results, points


url = "rtsp://192.168.1.2:554/stream"

cap = cv2.VideoCapture(url)
detector = cv2.QRCodeDetector()

qr_visible = False
last_seen_time = 0
disappear_delay = 2

qr_coordinates = []

while True:
    ret, frame = cap.read()

    if not ret:
        print("Couldn't receive frame")
        break

    results, points = scan_qr(frame, detector)
    current_time = time.time()

    if results:
        last_seen_time = current_time

        if not qr_visible:
            for data, x, y in results:
                if 0 <= x <= 3 and 0 <= y <= 3:
                    qr_coordinates.append((x, y))

                    print("New QR box detected!")
                    print("X:", x, "m")
                    print("Y:", y, "m")
                    print("Total boxes scanned:", len(qr_coordinates))
                    print("COOOOOOOOOOOOOOOOOOOOOOL")

            qr_visible = True

        if points is not None:
            points = points.astype(int)

            for qr_points in points:
                for i in range(4):
                    p1 = tuple(qr_points[i])
                    p2 = tuple(qr_points[(i + 1) % 4])
                    cv2.line(frame, p1, p2, (0, 0, 255), 3)

    elif qr_visible and current_time - last_seen_time >= disappear_delay:
        qr_visible = False

    cv2.imshow("QR Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

print("\nAll scanned destinations:")
for i, (x, y) in enumerate(qr_coordinates, start=1):
    print(f"Box {i}: X={x}, Y={y}")