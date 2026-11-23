# MID-sparse-dense

Compare RGB and sparse LiDAR with three LiDAR completion methods across 3,083 samples. Use **Previous** and **Next** to browse, or enter a number from 1 to 3,083 in **Go to frame** and press Enter. All four LiDAR maps use Viridis by default. Clear **Show Viridis Colormap** to switch to grayscale.

In **Sparse Range LiDAR**, use **Background** to show missing measurements in black or white.

## Open in your browser

[Open the viewer](https://kennedyk1.github.io/MID-sparse-dense/)

The viewer loads images directly from this repository on `raw.githubusercontent.com`.

[Open the interactive Bilateral Filter](https://kennedyk1.github.io/MID-sparse-dense/bilateral-filter.html)

The interactive page loads the original RGB image and LiDAR PLY for each frame. It projects the scan and computes the Bilateral Filter in your browser as you move the parameter sliders. Use **Restore defaults** to reset all four filter settings. Radial range is calculated from the PLY coordinates. [MID-3K camera and LiDAR calibration](https://github.com/kennedyk1/MID-3K/blob/main/calibration/color_lidar/calibration.yaml)

## Methods

- **BF** — Bilateral Filter, IEEE ITSC 2016. [Paper](https://doi.org/10.1109/ITSC.2016.7795953)
- **DTP** — Distance Transform Pooling, IEEE Transactions on Neural Networks and Learning Systems, 2023. [Paper](https://doi.org/10.1109/TNNLS.2021.3129801)
- **SGM** — Surface Geometry Model, IEEE Robotics and Automation Letters, 2021. [Paper](https://doi.org/10.1109/LRA.2021.3068885)
