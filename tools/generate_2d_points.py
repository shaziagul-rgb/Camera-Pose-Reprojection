#!/usr/bin/env python3
"""
Select 2D image points and save them as text files.

Each selected point is stored as:

    x,y

The order in which points are selected is preserved. This is important
because the C++ reprojection program uses the same point index to match
each 2D point with its corresponding 3D point.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


# Image formats supported by the point-selection tool.
SUPPORTED_EXTENSIONS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".tif",
    ".tiff",
}


def select_points(
    image_path: Path,
    output_path: Path,
) -> None:
    """
    Display an image and allow the user to manually select 2D points.

    Controls:
        Left mouse button : Add a point
        Backspace          : Remove the last point
        Enter / Space     : Save points and continue
        Esc                : Cancel the current image
    """

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if image is None:
        print(
            f"[Warning] Could not read image: "
            f"{image_path}"
        )
        return

    #
    # Store points in the order in which they are selected.
    #
    points: list[tuple[int, int]] = []

    window_name = (
        f"Select points: {image_path.name}"
    )

    #
    # Keep track of whether the user cancelled this image.
    #
    cancelled = False

    def redraw() -> None:
        """
        Redraw the image and all selected points.
        """

        display = image.copy()

        for index, (x, y) in enumerate(points):
            #
            # Draw the selected point.
            #
            cv2.circle(
                display,
                (x, y),
                5,
                (0, 255, 255),
                -1,
            )

            #
            # Display the point number next to the point.
            #
            cv2.putText(
                display,
                str(index + 1),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(
            window_name,
            display,
        )

    def click_event(
        event: int,
        x: int,
        y: int,
        _flags: int,
        _params: object,
    ) -> None:
        """
        Handle mouse clicks.
        """

        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))

            print(
                f"Selected point "
                f"{len(points)}: ({x}, {y})"
            )

            redraw()

    #
    # Create and display the image window.
    #
    cv2.namedWindow(
        window_name,
        cv2.WINDOW_NORMAL,
    )

    cv2.imshow(
        window_name,
        image,
    )

    cv2.setMouseCallback(
        window_name,
        click_event,
    )

    print()
    print("=" * 60)
    print(f"Image: {image_path.name}")
    print("=" * 60)
    print("Left click : add a 2D point")
    print("Backspace  : remove the last point")
    print("Enter      : save points and continue")
    print("Space      : save points and continue")
    print("Esc        : cancel this image")
    print()

    #
    # Wait for keyboard input.
    #
    while True:
        key = cv2.waitKey(0) & 0xFF

        #
        # Enter or Space:
        # save the selected points.
        #
        if key in (13, 32):
            break

        #
        # Backspace:
        # remove the most recently selected point.
        #
        if key in (8, 127):
            if points:
                removed_point = points.pop()

                print(
                    f"Removed point: "
                    f"{removed_point}"
                )

                redraw()
            else:
                print(
                    "No points to remove."
                )

        #
        # Escape:
        # cancel this image.
        #
        elif key == 27:
            cancelled = True
            break

    #
    # Close the image window.
    #
    cv2.destroyWindow(
        window_name
    )

    #
    # If the user cancelled, do not create an output file.
    #
    if cancelled:
        print(
            f"Cancelled: {image_path.name}"
        )
        return

    #
    # Create the output directory if it does not exist.
    #
    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    #
    # Save points in the order selected.
    #
    with output_path.open(
        "w",
        encoding="utf-8",
    ) as file:

        for x, y in points:
            file.write(
                f"{x},{y}\n"
            )

    print(
        f"Saved {len(points)} point(s) to "
        f"{output_path}"
    )


def main() -> None:
    """
    Parse command-line arguments and process all images.
    """

    parser = argparse.ArgumentParser(
        description=(
            "Interactively select 2D points "
            "from images."
        )
    )

    parser.add_argument(
        "--images",
        type=Path,
        default=Path("data/rgb"),
        help=(
            "Directory containing input images "
            "(default: data/rgb)"
        ),
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/2dPoints"),
        help=(
            "Directory for generated 2D point files "
            "(default: data/2dPoints)"
        ),
    )

    args = parser.parse_args()

    #
    # Check that the input image directory exists.
    #
    if not args.images.is_dir():
        raise SystemExit(
            f"Image directory does not exist: "
            f"{args.images}"
        )

    #
    # Find supported image files.
    #
    images = sorted(
        path
        for path in args.images.iterdir()
        if (
            path.is_file()
            and path.suffix.lower()
            in SUPPORTED_EXTENSIONS
        )
    )

    if not images:
        raise SystemExit(
            f"No supported images found in: "
            f"{args.images}"
        )

    print(
        f"Found {len(images)} image(s)."
    )

    #
    # Process each image in sorted order.
    #
    for image_path in images:

        output_path = (
            args.output
            / f"{image_path.name}.2dpoints.txt"
        )

        select_points(
            image_path,
            output_path,
        )


if __name__ == "__main__":
    main()