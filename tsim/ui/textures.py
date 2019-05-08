"""Module to load, cache and retrieve textures."""

from collections import namedtuple

from panda3d.core import SamplerState, Texture

import tsim.ui.panda3d as p3d


TextureInfo = namedtuple('TextureInfo', ['extension', 'filters'])

LOADED_TEXTURES = {}
PATH = 'textures/'


def get(texture_name: str) -> Texture:
    """Get texture with given name."""
    texture = LOADED_TEXTURES.get(texture_name, None)
    if texture is not None:
        return texture

    extension, filters = TEXTURES.get(texture_name, ('', ()))
    texture = p3d.LOADER.load_texture(''.join((PATH, texture_name, extension)),
                                      okMissing=True)
    if texture is not None:
        for filter_ in filters:
            filter_(texture)
        LOADED_TEXTURES[texture_name] = texture

    return texture


def _mipmap(texture: Texture):
    texture.set_minfilter(SamplerState.FT_linear_mipmap_linear)


TEXTURES = {
    'ground': TextureInfo('.jpg', (_mipmap,)),
    'intersection': TextureInfo('.png', ()),
    'road': TextureInfo('.png', (_mipmap,))
}
