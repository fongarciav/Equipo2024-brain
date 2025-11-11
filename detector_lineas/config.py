import platform
import cv2

LINUX_CAMERA_PATH = "/dev/video0"
LINUX_CAMERA_PATH_RGBD = "/dev/video4"
WINDOWS_CAMERA_PATH = 0


def detect_os():
    system = platform.system()
    if system == "Windows":
        return "windows"
    elif system == "Darwin":
        return "mac"
    elif "microsoft" in platform.uname().release.lower():  # Detect WSL
        return "wsl"
    else:
        return "linux"


def choose_camera_by_OS():
    os = detect_os()
    if os == "wsl" or os == "linux":
        return LINUX_CAMERA_PATH_RGBD
    else:
        return WINDOWS_CAMERA_PATH


def handle_video_capture(window_name, path):
    cv2.namedWindow(window_name)
    capture = cv2.VideoCapture(path)

    if not capture.isOpened():
        print("Error: Could not open video stream.")
        return None

    return capture