import cv2

url = "rtsp://192.168.1.2:554/stream"

cap = cv2.VideoCapture(url)

detector = cv2.QRCodeDetector()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Couldn't receive frame")
        break

    data, points, _ = detector.detectAndDecode(frame)

    if data:
        print("QR detected!")
        print("Data:", data)

    if points is not None:
        points = points.astype(int)

        for i in range(4):
            p1 = tuple(points[0][i])
            p2 = tuple(points[0][(i + 1) % 4])
            cv2.line(frame, p1, p2, (0, 0, 255), 3)

    cv2.imshow("iPhone Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()