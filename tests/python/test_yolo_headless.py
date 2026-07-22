#!/usr/bin/env python3
"""
Headless YOLOv8 test - saves annotated images instead of displaying
Works over SSH without display
"""

import os
import sys
import time

import cv2
from ultralytics import YOLO


def test_yolo_headless():
    print("=== Headless YOLOv8 Detection Test ===")

    # Create output directory
    output_dir = "yolo_test_output"
    os.makedirs(output_dir, exist_ok=True)

    # Load YOLO model
    print("\nLoading YOLOv8 model...")
    try:
        model = YOLO('yolov8n.pt')
        print("✅ YOLOv8n model loaded successfully!")
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
        return False

    # Open camera
    print("\nOpening camera...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Cannot open camera. Try running test_camera_headless.py first.")
        return False

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {width}x{height}")

    # Performance tracking
    print("\n📹 Running detection test (20 frames)...")
    frame_times = []
    detection_count = 0
    all_detections = []

    for i in range(20):
        start_time = time.time()

        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame {i+1}")
            continue

        # Run YOLO detection
        results = model(frame, verbose=False)

        # Get annotated frame
        annotated_frame = results[0].plot()

        # Calculate FPS
        elapsed = time.time() - start_time
        fps = 1 / elapsed if elapsed > 0 else 0
        frame_times.append(elapsed)

        # Count detections
        num_detections = len(results[0].boxes)
        if num_detections > 0:
            detection_count += 1

        # Add FPS overlay
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Detections: {num_detections}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Save every 4th frame
        if i % 4 == 0:
            filename = f"{output_dir}/detection_frame_{i+1:02d}.jpg"
            cv2.imwrite(filename, annotated_frame)
            print(f"  Frame {i+1}/20: {num_detections} detections, {fps:.1f} FPS - Saved to {filename}")
        else:
            print(f"  Frame {i+1}/20: {num_detections} detections, {fps:.1f} FPS")

        # Log detections
        if num_detections > 0:
            frame_detections = []
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]
                frame_detections.append(f"{label} ({conf:.2f})")
                print(f"    → {label} (confidence: {conf:.2f})")
            all_detections.append((i+1, frame_detections))

        time.sleep(0.1)  # Small delay between captures

    cap.release()

    # Print summary
    if frame_times:
        avg_fps = 1 / (sum(frame_times) / len(frame_times))
        print("\n✅ Test completed!")
        print(f"   Average FPS: {avg_fps:.1f}")
        print(f"   Frames with detections: {detection_count}/20")
        print(f"   Detection rate: {100*detection_count/20:.1f}%")
        print(f"   Annotated images saved in: {output_dir}/")

        # Summary of all detections
        if all_detections:
            print("\n📋 Detection Summary:")
            for frame_num, detections in all_detections:
                print(f"   Frame {frame_num}: {', '.join(detections)}")

        if avg_fps < 5:
            print("\n⚠️  Low FPS is normal on Raspberry Pi 4")
            print("   This is fine - we'll run detection at ~5-10 Hz")

        return True

    return False

if __name__ == "__main__":
    try:
        success = test_yolo_headless()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
