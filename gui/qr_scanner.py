
import cv2


def scan_qr(frame, detector):
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

                    coordinates[key.strip().upper()] = float(
                        value.strip()
                    )

                x = coordinates["X"]
                y = coordinates["Y"]

                if 0 <= x <= 3 and 0 <= y <= 3:
                    results.append((data, x, y))

            except (ValueError, KeyError):
                continue

    return results, points
