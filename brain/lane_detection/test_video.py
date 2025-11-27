import cv2
import numpy as np
import sys
import os
import argparse

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lane_detection.lane_detector import MarcosLaneDetector_Advanced
from lane_detection.pid_controller import PIDController
from lane_detection.angle_converter import AngleConverter

def draw_steering_visualization(servo_value, width=400, height=300):
    """
    Draw a top-down view of the car showing steering position.
    
    Args:
        servo_value: Servo angle (50=right, 105=center, 160=left)
        width, height: Size of the visualization
    
    Returns:
        Image showing car with steering wheel position
    """
    # Create black canvas
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Servo constants
    SERVO_CENTER = 105
    SERVO_RIGHT = 50
    SERVO_LEFT = 160
    
    # Convert servo value to steering angle (-1 to +1, where 0 is straight)
    # Normalize: -1 (full right) to +1 (full left)
    if servo_value < SERVO_CENTER:
        # Right turn
        normalized = -(SERVO_CENTER - servo_value) / (SERVO_CENTER - SERVO_RIGHT)
    elif servo_value > SERVO_CENTER:
        # Left turn
        normalized = (servo_value - SERVO_CENTER) / (SERVO_LEFT - SERVO_CENTER)
    else:
        # Straight
        normalized = 0.0
    
    # Clamp to [-1, 1]
    normalized = max(-1.0, min(1.0, normalized))
    
    # Car dimensions (top-down view)
    car_width = 80
    car_length = 140
    car_x = width // 2
    car_y = height // 2 + 20
    
    # Draw car body (rectangle)
    car_top_left = (car_x - car_width // 2, car_y - car_length // 2)
    car_bottom_right = (car_x + car_width // 2, car_y + car_length // 2)
    cv2.rectangle(img, car_top_left, car_bottom_right, (100, 100, 100), -1)
    cv2.rectangle(img, car_top_left, car_bottom_right, (200, 200, 200), 2)
    
    # Draw windshield (front of car)
    windshield_y = car_y - car_length // 2 + 15
    cv2.rectangle(img, 
                  (car_x - car_width // 3, car_y - car_length // 2),
                  (car_x + car_width // 3, windshield_y),
                  (150, 200, 255), -1)
    
    # Draw wheels (only front wheels will show steering)
    wheel_width = 12
    wheel_length = 30
    wheel_offset_x = car_width // 2 - 5
    
    # Front wheels (these will rotate)
    front_wheel_y = car_y - car_length // 2 + 35
    
    # Calculate wheel angle in radians (max 30 degrees)
    max_wheel_angle = np.radians(30)
    wheel_angle = normalized * max_wheel_angle
    
    # Draw front left wheel (rotated)
    draw_rotated_wheel(img, car_x - wheel_offset_x, front_wheel_y, 
                       wheel_width, wheel_length, wheel_angle, (50, 50, 50))
    
    # Draw front right wheel (rotated)
    draw_rotated_wheel(img, car_x + wheel_offset_x, front_wheel_y, 
                       wheel_width, wheel_length, wheel_angle, (50, 50, 50))
    
    # Draw rear wheels (straight)
    rear_wheel_y = car_y + car_length // 2 - 35
    cv2.rectangle(img,
                  (car_x - wheel_offset_x - wheel_width // 2, rear_wheel_y - wheel_length // 2),
                  (car_x - wheel_offset_x + wheel_width // 2, rear_wheel_y + wheel_length // 2),
                  (50, 50, 50), -1)
    cv2.rectangle(img,
                  (car_x + wheel_offset_x - wheel_width // 2, rear_wheel_y - wheel_length // 2),
                  (car_x + wheel_offset_x + wheel_width // 2, rear_wheel_y + wheel_length // 2),
                  (50, 50, 50), -1)
    
    # Draw direction arrow
    arrow_length = 80
    arrow_start = (car_x, car_y - car_length // 2 - 10)
    arrow_end_x = int(car_x + arrow_length * np.sin(normalized * max_wheel_angle))
    arrow_end_y = int(car_y - car_length // 2 - 10 - arrow_length * np.cos(normalized * max_wheel_angle))
    cv2.arrowedLine(img, arrow_start, (arrow_end_x, arrow_end_y), (0, 255, 0), 3, tipLength=0.3)
    
    # Draw servo value indicator
    servo_bar_width = 300
    servo_bar_height = 20
    servo_bar_x = (width - servo_bar_width) // 2
    servo_bar_y = height - 60
    
    # Background bar
    cv2.rectangle(img, 
                  (servo_bar_x, servo_bar_y),
                  (servo_bar_x + servo_bar_width, servo_bar_y + servo_bar_height),
                  (60, 60, 60), -1)
    
    # Center line
    center_x = servo_bar_x + servo_bar_width // 2
    cv2.line(img, (center_x, servo_bar_y), (center_x, servo_bar_y + servo_bar_height), (200, 200, 200), 2)
    
    # Servo position indicator
    servo_normalized = (servo_value - SERVO_RIGHT) / (SERVO_LEFT - SERVO_RIGHT)  # 0 to 1
    servo_x = int(servo_bar_x + servo_normalized * servo_bar_width)
    
    # Color based on direction
    if servo_value < SERVO_CENTER - 5:
        indicator_color = (0, 100, 255)  # Orange for right
    elif servo_value > SERVO_CENTER + 5:
        indicator_color = (255, 100, 0)  # Blue for left
    else:
        indicator_color = (0, 255, 0)  # Green for straight
    
    cv2.circle(img, (servo_x, servo_bar_y + servo_bar_height // 2), 12, indicator_color, -1)
    cv2.circle(img, (servo_x, servo_bar_y + servo_bar_height // 2), 12, (255, 255, 255), 2)
    
    # Text labels
    cv2.putText(img, f"Servo: {servo_value}", (width // 2 - 60, height - 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Direction labels
    if abs(normalized) < 0.1:
        direction = "STRAIGHT"
        dir_color = (0, 255, 0)
    elif normalized < 0:
        direction = f"RIGHT ({abs(normalized)*100:.0f}%)"
        dir_color = (0, 100, 255)
    else:
        direction = f"LEFT ({normalized*100:.0f}%)"
        dir_color = (255, 100, 0)
    
    cv2.putText(img, direction, (width // 2 - 70, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, dir_color, 2)
    
    return img

def draw_rotated_wheel(img, center_x, center_y, width, length, angle, color):
    """Draw a rotated wheel (rectangle)."""
    # Create rotation matrix
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    # Define wheel corners relative to center
    corners = np.array([
        [-width/2, -length/2],
        [width/2, -length/2],
        [width/2, length/2],
        [-width/2, length/2]
    ])
    
    # Rotate corners
    rotated_corners = np.zeros_like(corners)
    for i, corner in enumerate(corners):
        x, y = corner
        rotated_corners[i] = [
            x * cos_a - y * sin_a + center_x,
            x * sin_a + y * cos_a + center_y
        ]
    
    # Draw filled polygon
    pts = rotated_corners.astype(np.int32)
    cv2.fillPoly(img, [pts], color)
    cv2.polylines(img, [pts], True, (200, 200, 200), 1)

def draw_pause_indicator(img, paused, frame_count):
    """Draw pause indicator overlay on image."""
    h, w = img.shape[:2]
    
    # Semi-transparent overlay
    overlay = img.copy()
    if paused:
        # Red semi-transparent background
        cv2.rectangle(overlay, (0, 0), (w, 50), (0, 0, 200), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
        
        # Pause text
        text = "PAUSED - SPACE resume, RIGHT ARROW next, LEFT ARROW prev"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        text_x = (w - text_size[0]) // 2
        cv2.putText(img, text, (text_x, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    else:
        # Green semi-transparent background
        cv2.rectangle(overlay, (0, 0), (w, 30), (0, 200, 0), -1)
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)
        
        # Frame counter
        text = f"Frame: {frame_count}"
        cv2.putText(img, text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    parser = argparse.ArgumentParser(description='Test Lane Detection on Video')
    parser.add_argument('video_path', type=str, help='Path to the input video file')
    parser.add_argument('--threshold', type=int, default=150, help='Binary threshold for lane detection')
    args = parser.parse_args()

    if not os.path.exists(args.video_path):
        print(f"Error: Video file not found at {args.video_path}")
        return

    # Initialize modules
    detector = MarcosLaneDetector_Advanced(threshold=args.threshold)
    # Using parameters from verification
    pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.1, max_angle=30.0, deadband=1.0)
    converter = AngleConverter()

    cap = cv2.VideoCapture(args.video_path)
    
    # Get video properties for output
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    # Create video writer (DISABLED)
    # output_path = 'output_test.mp4'
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print(f"Processing video: {args.video_path}")
    print("Controls:")
    print("  'q' or ESC - Quit")
    print("  SPACE or 'p' - Pause/Resume")
    print("  RIGHT ARROW or 'n' - Next frame (when paused)")
    print("  LEFT ARROW or 'b' - Previous frame (when paused)")

    frame_count = 0
    paused = False
    
    # Read first frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read first frame from video")
        cap.release()
        cv2.destroyAllWindows()
        return
    
    frame_count += 1
    
    while cap.isOpened():
        # Only read new frame if not paused
        if not paused:
            ret, frame = cap.read()
            if not ret:
                break
            frame_count += 1
        
        # 1. Detect Lanes
        angle_deviation_deg, debug_images = detector.get_lane_metrics(frame)
        
        # If no lane detected, angle_deviation might be None or 0 depending on implementation
        # The current implementation returns None if no lanes found
        if angle_deviation_deg is None:
            angle_deviation_deg = 0.0
            lane_detected = False
        else:
            lane_detected = True

        # 2. PID Control
        # Assuming dt = 1/fps for simulation
        #dt = 1.0 / fps if fps > 0 else 0.033
        #control_signal = pid.compute(angle_deviation_deg, dt)

        # 3. Angle Conversion
        servo_value = converter.convert(angle_deviation_deg, inverted=True)

        # 4. Visualization
        # Show multiple views similar to dashboard
        
        # View 1: Final Result
        if debug_images and 'final_result' in debug_images:
            final_result = debug_images['final_result'].copy()
            draw_pause_indicator(final_result, paused, frame_count)
            cv2.imshow('Final Result', final_result)
        else:
            final_result = frame.copy()
            draw_pause_indicator(final_result, paused, frame_count)
            cv2.imshow('Final Result', final_result)

        # View 2: Bird View Lines
        if debug_images and 'bird_view_lines' in debug_images:
            bird_view = debug_images['bird_view_lines'].copy()
            draw_pause_indicator(bird_view, paused, frame_count)
            cv2.imshow('Bird View Lines', bird_view)

        # View 3: Sliding Windows
        if debug_images and 'sliding_windows' in debug_images:
            sliding_windows = debug_images['sliding_windows'].copy()
            draw_pause_indicator(sliding_windows, paused, frame_count)
            cv2.imshow('Sliding Windows', sliding_windows)
        
        # View 4: Steering Visualization
        steering_viz = draw_steering_visualization(servo_value)
        draw_pause_indicator(steering_viz, paused, frame_count)
        cv2.imshow('Steering Visualization', steering_viz)
        
        # Handle key presses
        # When paused, wait indefinitely (0 means wait forever)
        # When not paused, wait 1ms (normal playback speed)
        wait_time = 0 if paused else 1
        key = cv2.waitKey(wait_time) & 0xFF
        
        if key == ord('q') or key == 27:  # 'q' or ESC
            break
        elif key == ord(' ') or key == ord('p'):  # SPACE or 'p' to pause/resume
            paused = not paused
            if paused:
                print("Video PAUSED - SPACE resume, RIGHT ARROW next, LEFT ARROW previous")
            else:
                print("Video RESUMED")
        elif paused and (key == 83 or key == ord('n')):  # RIGHT ARROW (83) or 'n' for next frame
            # Advance one frame when paused - read and process immediately
            ret, frame = cap.read()
            if ret:
                frame_count += 1
                # Process this new frame immediately by continuing the loop
                continue
            else:
                # End of video
                break
        elif paused and (key == 81 or key == ord('b')):  # LEFT ARROW (81) or 'b' for previous frame
            # Move back one frame by setting the capture position
            target_frame = max(frame_count - 2, 0)
            cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
            ret, frame = cap.read()
            if ret:
                frame_count = target_frame + 1
                continue
            else:
                break
        
        if not paused and frame_count % 30 == 0:
            print(f"Processed {frame_count} frames...")

    cap.release()
    # out.release()
    cv2.destroyAllWindows()
    print("Done!")

if __name__ == "__main__":
    main()
