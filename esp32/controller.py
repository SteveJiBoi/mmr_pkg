import socket
import pygame
import math

# ==========================
# Robot Settings
# ==========================

ESP32_IP = "10.229.5.249"
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)          # so recv never freezes the control loop
last_reply = 0                   # time of last packet heard back from robot
connected = False

def check_replies():
    """Drain any ACK/PONG packets the robot sent back."""
    global last_reply, connected

    try:
        while True:
            data, addr = sock.recvfrom(64)

            last_reply = pygame.time.get_ticks()
            connected = True

            print(
                "robot:",
                data.decode(errors="replace").strip(),
                "from:",
                addr
            )

    except (BlockingIOError, ConnectionResetError):
        pass
speed = 180
rotation = 0
mode = "A"

pygame.init()
pygame.display.set_mode((300, 150))
pygame.display.set_caption("Omni Robot Controller")

clock = pygame.time.Clock()

print("Running...")

while True:

    pygame.event.pump()

    keys = pygame.key.get_pressed()

    x = 0
    y = 0

    # Translation vector
    if keys[pygame.K_w]:
        y += 1

    if keys[pygame.K_s]:
        y -= 1

    if keys[pygame.K_d]:
        x += 1

    if keys[pygame.K_a]:
        x -= 1

    # Rotation
    if keys[pygame.K_q]:
        rotation = -80

    elif keys[pygame.K_e]:
        rotation = 80

    else:
        rotation = 0

    # Speed
    if keys[pygame.K_UP]:
        speed = min(speed + 2, 255)

    if keys[pygame.K_DOWN]:
        speed = max(speed - 2, 0)

    # Calculate movement angle
    if x == 0 and y == 0:
        packet = f"A,0,0,{rotation}"
    else:

        angle = math.degrees(math.atan2(y, x))

        if angle < 0:
            angle += 360

        packet = f"A,{int(angle)},{speed},{rotation}"

    sock.sendto(packet.encode(), (ESP32_IP, PORT))
    check_replies()

    # time out the connection if no reply for 1 second
    if pygame.time.get_ticks() - last_reply > 1000:
        connected = False

    status = "CONNECTED" if connected else "NO LINK"
    pygame.display.set_caption(f"Omni Controller — {status} — {speed}")

    # optional heartbeat when idle so the link check still works while parked
    if x == 0 and y == 0 and rotation == 0:
        sock.sendto(b"PING", (ESP32_IP, PORT))
    print(packet, end="\r")

    if keys[pygame.K_ESCAPE]:
        break

    clock.tick(50)

pygame.quit()
