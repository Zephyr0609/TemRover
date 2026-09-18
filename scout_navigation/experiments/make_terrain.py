"""Heightmap matching the Phase-1 Simscape terrain recipe so both simulators see the same statistics."""
import numpy as np
from PIL import Image

TERRAIN_SIZE = 100.0
HEIGHT_SCALE = 2.0
RESOLUTION = 513
ROUGHNESS_RMS = {'C': 0.022, 'D': 0.045, 'E': 0.090}

SWELL_AMPLITUDE = (0.1, 0.6)
SWELL_WAVELENGTH = (30.0, 100.0)
RIPPLE_AMPLITUDE = (0.02, 0.15)
RIPPLE_WAVELENGTH = (5.0, 25.0)
WARP_AMPLITUDE = (0.02, 0.10)
WARP_WAVELENGTH_EAST = (150.0, 500.0)
WARP_WAVELENGTH_NORTH = (15.0, 60.0)


def fractal_roughness(shape, target_rms, rng):
    """ISO 8608 approximation: 1/f^1.5 amplitude spectrum with random phase."""
    frequencies = np.hypot(*np.meshgrid(np.fft.fftfreq(shape[0]), np.fft.fftfreq(shape[1]),
                                        indexing='ij'))
    frequencies[0, 0] = 1.0
    amplitude = frequencies ** -1.5
    amplitude[0, 0] = 0.0

    spectrum = amplitude * np.exp(2j * np.pi * rng.random(shape))
    surface = np.real(np.fft.ifft2(spectrum))
    return surface * target_rms / surface.std()


def rolling_swells(east, north, rng):
    """Long swell along track, lateral ripple, and a bounded three-dimensional warp."""
    swell = rng.uniform(*SWELL_AMPLITUDE) * np.sin(
        2.0 * np.pi * east / rng.uniform(*SWELL_WAVELENGTH) + rng.uniform(0.0, 2.0 * np.pi))
    ripple = rng.uniform(*RIPPLE_AMPLITUDE) * np.cos(
        2.0 * np.pi * north / rng.uniform(*RIPPLE_WAVELENGTH) + rng.uniform(0.0, 2.0 * np.pi))
    warp = rng.uniform(*WARP_AMPLITUDE) * np.sin(
        2.0 * np.pi * east / rng.uniform(*WARP_WAVELENGTH_EAST) + rng.uniform(0.0, 2.0 * np.pi)
    ) * np.sin(2.0 * np.pi * north / rng.uniform(*WARP_WAVELENGTH_NORTH)
               + rng.uniform(0.0, 2.0 * np.pi))
    return swell + ripple + warp


def main():
    rng = np.random.default_rng(0)
    axis = np.linspace(-TERRAIN_SIZE / 2.0, TERRAIN_SIZE / 2.0, RESOLUTION)
    east, north = np.meshgrid(axis, axis, indexing='ij')

    surface = fractal_roughness((RESOLUTION, RESOLUTION), ROUGHNESS_RMS['D'], rng)
    surface += rolling_swells(east, north, rng)
    surface -= surface.min()

    gradient = np.hypot(*np.gradient(surface, TERRAIN_SIZE / (RESOLUTION - 1)))
    print(f'peak-to-peak {surface.max():.3f} m, '
          f'slope max {np.degrees(np.arctan(gradient.max())):.1f} deg, '
          f'slope rms {np.degrees(np.arctan(gradient.std())):.1f} deg')

    Image.fromarray((surface / HEIGHT_SCALE * 255.0).astype(np.uint8)).save('worlds/terrain.png')
    Image.fromarray(np.tile(np.array([[[110, 120, 80]]], dtype=np.uint8), (256, 256, 1))).save(
        'worlds/terrain_texture.png')


if __name__ == '__main__':
    main()
