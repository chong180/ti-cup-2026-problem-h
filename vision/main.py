from maix import app, camera, display, err, gpio, image, nn, pinmap, time, touchscreen, uart

# YOLO11 model exported for MaixCAM2.
MODEL_PATH = "/root/models/test/best_yolo11.mud"
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.45

# Rod length used for the manual pixel-to-centimeter calibration.
ROD_LENGTH_CM = 25.0
MANUAL_CALIBRATION = False
FIXED_POINT_1 = (55, 252)
FIXED_POINT_2 = (639, 172)

# MaixCAM2 UART4: A21=TX, A22=RX, 115200 8N1.
UART_DEVICE = "/dev/ttyS4"
UART_BAUDRATE = 115200
SERIAL_PERIOD_MS = 50

# MaixCAM1 fill light: B3, active high.
USE_FILL_LIGHT = True
FILL_LIGHT_PIN = "B3"
FILL_LIGHT_GPIO = "GPIOB3"


def clamp(value, low, high):
    return max(low, min(high, value))


def touch_to_image(tx, ty, frame_w, frame_h, screen_w, screen_h):
    x = int(tx * frame_w / max(1, screen_w))
    y = int(ty * frame_h / max(1, screen_h))
    return clamp(x, 0, frame_w - 1), clamp(y, 0, frame_h - 1)


def draw_text(img, x, y, text, color=image.COLOR_RED):
    img.draw_string(int(x), int(y), text, color=color, scale=1)


def send_frame(serial_dev, frame):
    if serial_dev is not None:
        serial_dev.write_str(frame)


def init_fill_light(enabled):
    try:
        err.check_raise(
            pinmap.set_pin_function(FILL_LIGHT_PIN, FILL_LIGHT_GPIO),
            "set fill light pin failed",
        )
        light = gpio.GPIO(FILL_LIGHT_GPIO, gpio.Mode.OUT)
        light.value(1 if enabled else 0)
        print("Fill light:", "on" if enabled else "off")
        return light
    except Exception as exc:
        print("Fill light init failed:", exc)
        return None


def shutdown_fill_light(light):
    if light is None:
        return
    try:
        light.value(0)
        print("Fill light: off")
    except Exception as exc:
        print("Fill light shutdown failed:", exc)


detector = nn.YOLO11(
    model=MODEL_PATH,
    dual_buff=True,
)

print("YOLO11 model loaded")
print("Labels:", detector.labels)

cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)

disp = display.Display()
screen_w = disp.width()
screen_h = disp.height()
ts = touchscreen.TouchScreen()
fill_light = init_fill_light(USE_FILL_LIGHT)

try:
    err.check_raise(
        pinmap.set_pin_function("A21", "UART4_TX"),
        "Failed to set A21 as UART4_TX",
    )
    err.check_raise(
        pinmap.set_pin_function("A22", "UART4_RX"),
        "Failed to set A22 as UART4_RX",
    )
    serial_dev = uart.UART(UART_DEVICE, UART_BAUDRATE)
    print("UART ready:", UART_DEVICE, UART_BAUDRATE)
except Exception as exc:
    serial_dev = None
    print("UART open failed:", exc)


calibration_points = []
rod_left = None
rod_right = None
rod_top = None
rod_bottom = None
touch_down = False

last_serial_ms = time.ticks_ms()

if not MANUAL_CALIBRATION:
    x1, y1 = FIXED_POINT_1
    x2, y2 = FIXED_POINT_2
    calibration_points = [FIXED_POINT_1, FIXED_POINT_2]
    rod_left = min(x1, x2)
    rod_right = max(x1, x2)
    rod_top = min(y1, y2)
    rod_bottom = max(y1, y2)
    print(
        "Fixed calibration loaded:",
        "left=", rod_left,
        "right=", rod_right,
        "top=", rod_top,
        "bottom=", rod_bottom,
    )

if MANUAL_CALIBRATION:
    print("Calibration required: click rod top-left, then bottom-right")
else:
    print("Calibration mode: fixed points", FIXED_POINT_1, FIXED_POINT_2)

try:
    while not app.need_exit():
        img = None
        try:
            img = cam.read()
        except Exception as exc:
            print("camera read retry:", exc)
            time.sleep_ms(10)
            continue

        if img is None:
            time.sleep_ms(1)
            continue

        if MANUAL_CALIBRATION:
            tx, ty, pressed = ts.read()
            if pressed and not touch_down:
                px, py = touch_to_image(
                    tx, ty, img.width(), img.height(), screen_w, screen_h
                )
                if len(calibration_points) < 2:
                    calibration_points.append((px, py))
                    print("Calibration point", len(calibration_points), px, py)
                    if len(calibration_points) == 2:
                        x1, y1 = calibration_points[0]
                        x2, y2 = calibration_points[1]
                        rod_left = min(x1, x2)
                        rod_right = max(x1, x2)
                        rod_top = min(y1, y2)
                        rod_bottom = max(y1, y2)

                        if rod_right - rod_left < 10 or rod_bottom - rod_top < 10:
                            print("Calibration points are too close; please retry")
                            calibration_points = []
                            rod_left = rod_right = rod_top = rod_bottom = None
                        else:
                            print(
                                "Calibration complete:",
                                "left=", rod_left,
                                "right=", rod_right,
                                "top=", rod_top,
                                "bottom=", rod_bottom,
                            )
                touch_down = True
            elif not pressed:
                touch_down = False

        if rod_left is None:
            if len(calibration_points) == 0:
                draw_text(img, 8, 8, "CALIBRATION: click rod TOP-LEFT")
            else:
                x1, y1 = calibration_points[0]
                img.draw_circle(x1, y1, 5, color=image.COLOR_RED, thickness=2)
                draw_text(img, 8, 8, "CALIBRATION: click rod BOTTOM-RIGHT")
                draw_text(img, 8, 26, "First point: (%d,%d)" % (x1, y1))

            for px, py in calibration_points:
                img.draw_circle(px, py, 5, color=image.COLOR_RED, thickness=2)
            disp.show(img)
            time.sleep_ms(1)
            continue

        center_x = (rod_left + rod_right) / 2.0

        objects = detector.detect(
            img,
            conf_th=CONF_THRESHOLD,
            iou_th=IOU_THRESHOLD,
        )

        ball = None
        best_score = -1.0
        for obj in objects:
            cx = float(obj.x) + float(obj.w) / 2.0
            cy = float(obj.y) + float(obj.h) / 2.0
            if rod_left <= cx <= rod_right and rod_top <= cy <= rod_bottom:
                if float(obj.score) > best_score:
                    ball = obj
                    best_score = float(obj.score)

        position_cm = None
        if ball is not None:
            x = int(ball.x)
            y = int(ball.y)
            w = int(ball.w)
            h = int(ball.h)
            ball_cx = x + w / 2.0
            ball_cy = y + h / 2.0

            # Absolute X-position error relative to the calibration-box center.
            # Left of center is positive; right of center is negative.
            position_cm = (center_x - ball_cx) / (rod_right - rod_left) * ROD_LENGTH_CM
            position_cm = clamp(position_cm, -ROD_LENGTH_CM / 2.0, ROD_LENGTH_CM / 2.0)

            img.draw_rect(x, y, w, h, color=image.COLOR_RED, thickness=2)
            img.draw_circle(int(ball_cx), int(ball_cy), 4, color=image.COLOR_RED, thickness=2)

            label = (
                detector.labels[ball.class_id]
                if 0 <= ball.class_id < len(detector.labels)
                else str(ball.class_id)
            )
            draw_text(img, x, max(0, y - 18), "%s: %.2f" % (label, ball.score))

            print(
                "class=%s score=%.3f box=(%d,%d,%d,%d) center_x=%.1f ball_x=%.1f x_error_cm=%+.2f"
                % (label, ball.score, x, y, w, h, center_x, ball_cx, position_cm)
            )

        img.draw_rect(
            rod_left,
            rod_top,
            rod_right - rod_left,
            rod_bottom - rod_top,
            color=image.COLOR_GREEN,
            thickness=2,
        )
        img.draw_line(
            int(center_x),
            rod_top,
            int(center_x),
            rod_bottom,
            color=image.COLOR_RED,
            thickness=2,
        )

        draw_text(img, 8, 8, "FPS: %.1f" % time.fps())
        if position_cm is None:
            draw_text(img, 8, 26, "Ball: -- cm")
        else:
            draw_text(img, 8, 26, "CX:%.1f BX:%.1f" % (center_x, ball_cx))
            draw_text(img, 8, 44, "Xerr(L+):%+.2fcm" % position_cm)

        now_ms = time.ticks_ms()
        #if time.ticks_diff(now_ms, last_serial_ms) >= SERIAL_PERIOD_MS:
        if position_cm is None:
            tx_frame = "#N!\r\n"
        else:
            tx_frame = "#%.2f,%.2f!\r\n" % (center_x, ball_cx)
        send_frame(serial_dev, tx_frame)
        print("UART TX:", tx_frame.strip())
        #last_serial_ms = now_ms

        disp.show(img)
        time.sleep_ms(1)
finally:
    shutdown_fill_light(fill_light)
