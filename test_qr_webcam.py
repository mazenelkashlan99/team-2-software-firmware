
import cv2
import time

# Open laptop webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Couldn't open laptop camera!")
    exit()

detector = cv2.QRCodeDetector()

qr_visible = False
last_seen_time = 0
disappear_delay = 2

qr_coordinates = []

print("QR Scanner started. Press Q to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Couldn't receive frame")
        break

    success, decoded_info, points, _ = (
        detector.detectAndDecodeMulti(frame)
    )

    results = []

    if success:
        for data in decoded_info:
            if not data:
                continue

            try:
                coordinates = {}

                for part in data.split("&"):
                    key, separator, value = part.partition("=")

                    if not separator:
                        raise ValueError("Invalid QR format")

                    coordinates[key.strip().upper()] = float(value.strip())

                x = coordinates["X"]
                y = coordinates["Y"]

                if 0 <= x <= 3 and 0 <= y <= 3:
                    results.append((data, x, y))

            except (ValueError, KeyError):
                print("QR found, but format is invalid:", data)

    current_time = time.time()

    if results:
        last_seen_time = current_time

        if not qr_visible:
            for data, x, y in results:
                qr_coordinates.append((x, y))

                print("\nNew QR box detected!")
                print("X:", x, "m")
                print("Y:", y, "m")
                print("Total boxes scanned:", len(qr_coordinates))

            qr_visible = True

        # Draw QR boundaries
        if points is not None:
            points = points.astype(int)

            for qr_points in points:
                for i in range(4):
                    p1 = tuple(qr_points[i])
                    p2 = tuple(qr_points[(i + 1) % 4])

                    cv2.line(frame, p1, p2, (0, 0, 255), 3)

        # Display coordinates
        for _, x, y in results:
            cv2.putText(
                frame,
                f"X: {x:.2f} m, Y: {y:.2f} m",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

    elif qr_visible and current_time - last_seen_time >= disappear_delay:
        qr_visible = False

    cv2.imshow("Laptop QR Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

print("\nAll scanned destinations:")

for i, (x, y) in enumerate(qr_coordinates, start=1):
    print(f"Box {i}: X={x}, Y={y}")
