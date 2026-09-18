"""Reads the survey parameters the navigator is actually launched with."""
import pathlib

import yaml

CONFIG_DIRECTORY = pathlib.Path(__file__).resolve().parent.parent / 'config'
BASE_CONFIG = CONFIG_DIRECTORY / 'survey.yaml'
GRID_KEYS = {'width': 'grid_width', 'length': 'grid_length', 'line_spacing': 'line_spacing',
             'turn_radius': 'turn_radius', 'point_spacing': 'point_spacing'}


def parameters(site=None):
    """Base parameters with a site file layered on top, exactly as the launch file passes them."""
    values = yaml.safe_load(BASE_CONFIG.read_text())['/**']['ros__parameters']
    if site is not None:
        values.update(yaml.safe_load(pathlib.Path(site).read_text())['/**']['ros__parameters'])
    return values


def load_grid(site=None):
    values = parameters(site)
    return {name: values[key] for name, key in GRID_KEYS.items()}


def grid_bearing(site=None):
    return parameters(site)['grid_bearing']


def load_obstacles():
    """The same layout the launch file spawns, so the recorder scores what is actually in the world."""
    return yaml.safe_load((CONFIG_DIRECTORY / 'obstacles.yaml').read_text())['obstacles']
