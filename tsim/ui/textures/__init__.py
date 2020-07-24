"""Module to load, cache and retrieve textures."""

from __future__ import annotations

from importlib import import_module
from io import BytesIO
from typing import Callable, NamedTuple, Sequence

from panda3d.core import PNMImage, SamplerState, StringStream, Texture
from PIL import Image

import tsim.ui.panda3d as p3d

LOADED_TEXTURES = {}
PATH = 'textures/'


class TextureInfo(NamedTuple):
    """Texture information."""

    extension: str
    filters: Sequence[Callable[[Texture], None]]


def _mipmap(texture: Texture):
    texture.set_minfilter(SamplerState.FT_linear_mipmap_linear)


def _anisotropic(degree: int):
    def _apply_anisotropic(texture: Texture):
        texture.set_anisotropic_degree(degree)
    return _apply_anisotropic


TEXTURES = {
    'ground': TextureInfo('.jpg', (_mipmap, _anisotropic(4))),
    'intersection': TextureInfo('.png', ()),
    'road': TextureInfo('.png', (_mipmap, _anisotropic(4)))
}


def get(texture_name: str) -> Texture:
    """Get texture with given name."""
    texture = LOADED_TEXTURES.get(texture_name, None)
    if texture is not None:
        return texture

    extension, filters = TEXTURES.get(texture_name, ('', ()))
    texture = p3d.LOADER.load_texture(''.join((PATH, texture_name, extension)),
                                      okMissing=True)

    if texture is None:
        factory = import_module(f'.{texture_name}', 'tsim.ui.textures')
        texture = create_texture(factory.create())

    if texture is not None:
        for filter_ in filters:
            filter_(texture)
        LOADED_TEXTURES[texture_name] = texture

    return texture


def create_texture(image: Image) -> Texture:
    """Create a Panda3D Texture from a PIL Image."""
    bytes_io = BytesIO()
    image.save(bytes_io, format='PNG')
    bytes_io.seek(0)
    stream = StringStream()
    stream.set_data(bytes_io.read())
    bytes_io.close()
    pnm_image = PNMImage()
    pnm_image.read(stream)
    stream.clear_data()
    texture = Texture()
    texture.load(pnm_image)
    pnm_image.clear()
    return texture
