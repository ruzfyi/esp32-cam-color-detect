# ESP32 CAM COLOR DETECT

Color detect algorithm. Converts the RGB88 image into HSV and assesses in that format. Pins are set up for the AI Thinker camera module.

## Threshold Table

Organized in the `Hmin, Hmax, Smin, Smax, Vmin, Vmax` format format. Singles out Red1, Red2, Green, Blue, Purple. Red is split in two because it takes up the top edge and bottom edge of the Hue values.