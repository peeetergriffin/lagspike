# Lagspike!
A Geode mod for Geometry Dash that displays real-time physics stability metrics.

## What It Does
Shows an on-screen overlay with:
- **Physics Stability Graph** - Visual display of frame timing consistency
- **Jitter Value** - Measures variance in physics frames (lower = smoother)
- **Stability %** - Percentage of frames within target timing
- **Engine Status** - Green (stable) or Red (jittering)

## Technical Details
- Tracks all 240 physics frames per second
- Calculates jitter using standard deviation
- Microsecond-precision timing

## Credits
- [Slideglide](https://github.com/slideglide) - Major performance improvements and overall help

## License
This project is created by [Gryffin](https://github.com/peeetergriffin) under the [GNU General Public License v3.0](https://choosealicense.com/licenses/gpl-3.0/)