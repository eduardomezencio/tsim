"""Screen capturing tools."""
import logging as log
import os
from datetime import datetime
from pathlib import Path

from tsim.ui import panda3d as p3d

PATH = Path.home().joinpath('.tsim', 'screen')


class Screen:
    """Screen capturing tools."""

    def __init__(self):
        PATH.mkdir(parents=True, exist_ok=True)
        self._movie_task = None

    def start_movie_recording(self, stop_current: bool = True):
        """Start recording movie from app window.

        A folder named with a timestamp is created each time a new recording
        starts.
        """
        if stop_current:
            self.stop_movie_recording()
        elif self._movie_task is not None:
            return

        dir_ = str(PATH.joinpath(_timestamp_str()))
        os.mkdir(dir_)
        self._movie_task = p3d.base.movie(f'{dir_}/', 31536000, 60, 'jpg')
        log.info('Started recording movie to %s.', dir_)

    def stop_movie_recording(self):
        """Stop movie recording, if one is in progress."""
        if self._movie_task is not None:
            self._movie_task.remove()
            self._movie_task = None
            log.info('Stopped recording movie.')

    def toggle_movie_recording(self):
        """Start or stop recording movie from app window."""
        if self._movie_task is None:
            self.start_movie_recording()
        else:
            self.stop_movie_recording()

    def screenshot(self, format_: str = 'png'):
        """Take screenshot."""
        p3d.base.screenshot(str(PATH.joinpath(f'{_timestamp_str()}.{format_}')), False)


def _timestamp_str() -> str:
    return str(datetime.now()).replace(" ", "_")
