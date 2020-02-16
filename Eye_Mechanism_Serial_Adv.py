#!/usr/bin/env python3

import argparse
import serial
import time

from aiy.vision.inference import CameraInference
from aiy.vision.models import face_detection
from examples.vision.annotator import Annotator
from picamera import PiCamera
from gpiozero import Button


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--num_frames',
        '-n',
        type=int,
        dest='num_frames',
        default=-1,
        help='Sets the number of frames to run for, otherwise runs forever.')
    args = parser.parse_args()
    ser = serial.Serial('/dev/ttyACM0', 115200, write_timeout=0.03)
    ser.write(b"<0.0,0.0,0.0,0.0>")

    with PiCamera() as camera:
        camera.sensor_mode = 4
        camera.resolution = (1640, 1232)
        camera.framerate = 30
        camera.start_preview()
        global shutdown_bool
        shutdown_bool = False
        position = 0
        NoFaceCount = 0
        NoFaceReset = 60
        NoFaceShut = 120
        Top_Lid_Offset = 25  # Top Lid = L2
        Bottom_Lid_Offset = 25  # Bottom Lid = L1
        Top_Lid_Limit = 45
        Bottom_Lid_Limit= 45
        step_size = 2.0

        X_Degrees = 0
        Y_Degrees = 0
        L1_Degrees = 0
        L2_Degrees = 0

        # Trying to slow down serial writing
        Write_Delay = 5  # Milliseconds
        Last_Write_Time = time.time() * 1000  # Milliseconds

        # Soft shutdown function
        def soft_shutdown():
            # X_Degrees = 0
            # Y_Degrees = 0
            # L1_Degrees = -60
            # L2_Degrees = -60
            # ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
            # camera.stop_preview()
            # quit()
            # thread.interrupt_main()
            print("Shutdown Initiated")
            global shutdown_bool
            shutdown_bool = True

        # Add shutdown function to button
        button = Button(23)
        button.when_pressed = soft_shutdown #pretty sure this doesn't work

        annotator = Annotator(camera, dimensions=(320, 240))
        scale_x = 320 / 1640
        scale_y = 240 / 1232

        # Transform is a function to transform the face bounding box. Incoming boxes are of the form (x, y, width, height). Scale and
        # transform to the form (x1, y1, x2, y2). X and Y are at the top left corner of the face bounding box.
        def transform(bounding_box):
            x, y, width, height = bounding_box
            return (scale_x * x, scale_y * y, scale_x * (x + width),
                    scale_y * (y + height))

        def x_y_to_angles(x, y):
            Span_X = 1640  # This is hard-coded resolution
            Span_Y = 1232
            MinDeg_X = 23  # These are hard coded servo angles, will need to be adjusted to match camera FOV
            MaxDeg_X = -23
            MinDeg_Y = -18
            MaxDeg_Y = 19

            SpanDeg_X = MaxDeg_X - MinDeg_X
            X_Degrees = MinDeg_X + (SpanDeg_X * (x / Span_X))

            SpanDeg_Y = MaxDeg_Y - MinDeg_Y
            Y_Degrees = MinDeg_Y + (SpanDeg_Y * (y / Span_Y))

            return X_Degrees, Y_Degrees

        with CameraInference(face_detection.model()) as inference:
            for i, result in enumerate(inference.run()):
                if i == args.num_frames:
                    break
                if shutdown_bool is True:
                    X_Degrees = 0
                    Y_Degrees = 0
                    L1_Degrees = 0
                    L2_Degrees = 0
                    ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                    break
                faces = face_detection.get_faces(result)
                annotator.clear()
                for face in faces:
                    annotator.bounding_box(transform(face.bounding_box), fill=0)  # adding bounding box to preview
                annotator.update()
                print('Iteration #%d: num_faces=%d' % (i, len(faces)))

                if faces:  # Give servo commands if a face is detected
                    face = faces[0]
                    x_corner, y_corner, width, height = face.bounding_box  # bounding box is top left corner
                    x = x_corner + width / 2
                    y = y_corner + height / 2
                    print('             : Face is at X = %d' % x)
                    print('             : Face is at Y = %d' % y)

                    Current_Time = time.time() * 1000
                    if Current_Time >= (Last_Write_Time + Write_Delay):
                        X_Degrees, Y_Degrees = x_y_to_angles(x, y)

                        L1_Degrees = min(Y_Degrees + Bottom_Lid_Offset, Bottom_Lid_Limit)
                        L2_Degrees = min(-Y_Degrees + Top_Lid_Offset, Top_Lid_Limit)

                        ser.cancel_write()  # Cancels previous write
                        ser.reset_output_buffer()  # Removes any data that hasn't been sent yet
                        ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        NoFaceCount = 0


                        # print('            : X Servo Angle =%d' % X_Degrees)
                        # print('            : Y Servo Angle =%d' % Y_Degrees)
                        print(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        Last_Write_Time = time.time() * 1000
                    else:
                        print("Waiting for Write Delay")

                else:
                    print('NoFaceCount = %d' % NoFaceCount)
                    if NoFaceReset <= NoFaceCount < NoFaceShut:
                        X_Degrees = 0
                        Y_Degrees = 0
                        L1_Degrees = 12
                        L2_Degrees = 12
                        ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        NoFaceCount = NoFaceCount + 1

                    if NoFaceCount >= NoFaceShut:
                            L1_Degrees = 0
                            L2_Degrees = 0
                            ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                            # NoFaceCount = NoFaceCount + 1

                    else:
                        NoFaceCount = NoFaceCount + 1

        camera.stop_preview()


if __name__ == '__main__':
    main()
