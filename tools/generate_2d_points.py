#!/usr/bin/env python3
"""Interactively select 2D image points and save them as CSV-style text files."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def select_points(image_path: Path, output_path: Path) -> None:
    image = cv2.imread(str(image_path))
    if image is None:
        print(f"[Warning] Could not read image: {image_path}")
        return

    points: list[tuple[int, int]] = []

    window_name = f"Select points: {image_path.name}"

    def click_event(event, x, y, _flags, _params):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            print(f"Selected point: ({x}, {y})")

            display = image.copy()
            for px, py in points:
                cv2.circle(display, (px, py), 5, (0, 255, 255), -1)
            cv2.imshow(window_name, display)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image)
    cv2.setMouseCallback(window_name, click_event)

    print(f"\nImage: {image_path.name}")
    print("  Left click: add a 2D point")
    print("  Press any key: save points and continue")
    cv2.waitKey(0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file:
        for x, y in points:
            file.write(f"{x},{y}\n")

    print(f"Saved {len(points)} point(s) to {output_path}")

    cv2.destroyWindow(window_name)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactively select 2D points from images."
    )
    parser.add_argument(
        "--images",
        type=Path,
        default=Path("data/rgb"),
        help="Directory containing input images (default: data/rgb)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/2dPoints"),
        help="Directory for generated 2D point files (default: data/2dPoints)",
    )
    args = parser.parse_args()

    if not args.images.is_dir():
        raise SystemExit(f"Image directory does not exist: {args.images}")

    images = sorted(
        p for p in args.images.iterdir()
        if p.is_file() and p.suffix.lower() in SUPPORTED_EXTENSIONS
    )

    if not images:
        raise SystemExit(f"No supported images found in: {args.images}")

    for image_path in images:
        output_path = args.output / f"{image_path.name}.2dpoints.txt"
        select_points(image_path, output_path)


if __name__ == "__main__":
    main()
