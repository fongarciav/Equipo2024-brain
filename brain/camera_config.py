import platform
import cv2
import os

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


def try_open_camera(camera_path):
    """
    Try to open a camera and verify it works.
    
    Args:
        camera_path: Path to camera device
        
    Returns:
        cv2.VideoCapture object if successful, None otherwise
    """
    try:
        # Check if device exists (Linux only)
        if isinstance(camera_path, str) and camera_path.startswith("/dev/"):
            if not os.path.exists(camera_path):
                return None
        
        cap = cv2.VideoCapture(camera_path)
        if cap.isOpened():
            # Try to read a frame to verify it actually works
            ret, _ = cap.read()
            if ret:
                return cap
            else:
                cap.release()
                return None
        else:
            return None
    except Exception:
        return None


def choose_camera_by_OS():
    """
    Choose camera path based on OS.
    For Linux/WSL, tries video0 first, then video4 if video0 fails.
    
    Returns:
        Camera path (string or int)
    """
    os_type = detect_os()
    
    if os_type == "wsl" or os_type == "linux":
        # Try video0 first
        test_cap = try_open_camera(LINUX_CAMERA_PATH)
        if test_cap is not None:
            test_cap.release()
            print(f"[camera_config] Using {LINUX_CAMERA_PATH}")
            return LINUX_CAMERA_PATH
        else:
            # Fall back to video4 (Intel RealSense)
            test_cap = try_open_camera(LINUX_CAMERA_PATH_RGBD)
            if test_cap is not None:
                test_cap.release()
                print(f"[camera_config] Using {LINUX_CAMERA_PATH_RGBD} (fallback)")
                return LINUX_CAMERA_PATH_RGBD
            else:
                # If both fail, return video0 as default (will show error later)
                print(f"[camera_config] Warning: Neither video0 nor video4 available, defaulting to {LINUX_CAMERA_PATH}")
                return LINUX_CAMERA_PATH
    else:
        return WINDOWS_CAMERA_PATH


def handle_video_capture(window_name, path):
    cv2.namedWindow(window_name)
    capture = cv2.VideoCapture(path)

    if not capture.isOpened():
        print("Error: Could not open video stream.")
        return None

    return capture