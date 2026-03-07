"""
USB webcam capture for ARGOS.

Wraps OpenCV VideoCapture. Tries the given device index (default 0);
raises RuntimeError if the device cannot be opened or returns no frames.

Usage::

    from argos.vision.camera import Camera

    cam = Camera()
    frame = cam.capture()    # numpy uint8 BGR array, shape (H, W, 3)
    cam.close()

    # or as a context manager:
    with Camera() as cam:
        frame = cam.capture()
"""

import cv2


class Camera:
    """
    USB webcam wrapper.

    Parameters
    ----------
    index : int
        V4L2 device index. 0 = /dev/video0 (default).
    width, height : int
        Requested resolution. The driver may round to the nearest
        supported mode; call actual_resolution() to confirm.
    fps : int
        Requested frame rate. Same caveats as width/height.
    """

    def __init__(self, index=0, width=640, height=480, fps=30):
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera at index {index}. "
                "Check 'ls /dev/video*' and try a different index."
            )
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)

    def capture(self):
        """
        Grab and decode one frame.

        Returns
        -------
        numpy.ndarray
            BGR image, shape (H, W, 3), dtype uint8.

        Raises
        ------
        RuntimeError
            If the camera fails to return a frame.
        """
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera returned no frame.")
        return frame

    def actual_resolution(self):
        """Return (width, height) as reported by the driver."""
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def actual_fps(self):
        """Return the frame rate as reported by the driver."""
        return self._cap.get(cv2.CAP_PROP_FPS)

    def close(self):
        """Release the capture device."""
        if self._cap.isOpened():
            self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
