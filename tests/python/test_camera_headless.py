#!/usr/bin/env python3
"""
Headless camera test - saves images instead of displaying
Works over SSH without display
"""

import os
import sys
import time

import cv2


def test_camera_headless():
    print("=== Headless Camera Test ===")
    print("Testing camera connection (no display required)...")

    # Create output directory
    output_dir = "camera_test_output"
    os.makedirs(output_dir, exist_ok=True)

    # Try different camera indices
    for camera_id in [0, 1]:
        print(f"\nTrying camera index {camera_id}...")
        cap = cv2.VideoCapture(camera_id)

        if not cap.isOpened():
            print(f"  ❌ Camera {camera_id} not accessible")
            continue

        # Get camera properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        print(f"  ✅ Camera {camera_id} opened successfully!")
        print(f"     Resolution: {width}x{height}")
        print(f"     FPS: {fps}")

        # Capture test frames
        print("\n  Capturing test frames...")
        for i in range(5):
            ret, frame = cap.read()

            if not ret:
                print(f"    ❌ Failed to capture frame {i+1}")
                continue

            # Save frame
            filename = f"{output_dir}/camera_{camera_id}_frame_{i+1}.jpg"
            cv2.imwrite(filename, frame)
            print(f"    ✅ Saved: {filename}")
            time.sleep(0.5)

        # Capture short video
        print("\n  Recording 5-second video...")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_filename = f"{output_dir}/camera_{camera_id}_test.mp4"
        out = cv2.VideoWriter(video_filename, fourcc, 20.0, (width, height))

        start_time = time.time()
        frame_count = 0

        while time.time() - start_time < 5.0:
            ret, frame = cap.read()
            if ret:
                out.write(frame)
                frame_count += 1

        out.release()
        print(f"    ✅ Saved {frame_count} frames to: {video_filename}")

        cap.release()

        print("\n✅ Camera test completed successfully!")
        print(f"   Working camera index: {camera_id}")
        print(f"   Test files saved in: {output_dir}/")
        print("\n   Review the images/video to verify camera is working properly.")

        return True

    print("\n❌ No working camera found!")
    print("\nTroubleshooting:")
    print("1. Check camera cable connection")
    print("2. Enable camera: sudo raspi-config -> Interface Options -> Camera")
    print("3. Reboot: sudo reboot")
    print("4. Check if camera is detected: vcgencmd get_camera")
    return False

if __name__ == "__main__":
    try:
        success = test_camera_headless()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
