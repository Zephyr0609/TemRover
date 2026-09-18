"""Stacks two recorded runs side by side into one video, each frame labelled, for a before/after slide."""
import sys

import cv2

LABEL_HEIGHT = 44
FONT = cv2.FONT_HERSHEY_SIMPLEX


def labelled(frame, label):
    """Adds a dark title band above the frame."""
    band = frame[:LABEL_HEIGHT].copy()
    band[:] = (28, 28, 28)
    cv2.putText(band, label, (14, 30), FONT, 0.9, (240, 240, 240), 2, cv2.LINE_AA)
    return cv2.vconcat([band, frame])


def main():
    left, right, output, left_label, right_label = sys.argv[1:6]
    readers = [cv2.VideoCapture(left), cv2.VideoCapture(right)]
    rate = readers[0].get(cv2.CAP_PROP_FPS)
    frames = int(min(reader.get(cv2.CAP_PROP_FRAME_COUNT) for reader in readers))
    width = int(readers[0].get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(readers[0].get(cv2.CAP_PROP_FRAME_HEIGHT)) + LABEL_HEIGHT

    writer = cv2.VideoWriter(output, cv2.VideoWriter_fourcc(*'mp4v'), rate, (2 * width, height))
    for _ in range(frames):
        pair = [reader.read()[1] for reader in readers]
        writer.write(cv2.hconcat([labelled(pair[0], left_label), labelled(pair[1], right_label)]))
    writer.release()
    print(f'wrote {output}: {frames} frames, {frames / rate:.0f} s, {2 * width}x{height}')


if __name__ == '__main__':
    main()
