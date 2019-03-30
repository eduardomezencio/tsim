"""Module to load, cache and retrieve textures."""

from collections import namedtuple

from direct.showbase.Loader import Loader
from panda3d.core import SamplerState, Texture


TextureInfo = namedtuple('TextureInfo', ['extension', 'filters'])

LOADED_TEXTURES = {}
PATH = 'textures/'


def get(texture_name: str, loader: Loader = None) -> Texture:
    """Get texture with given name."""
    if loader is None:
        loader = get.loader

    texture = LOADED_TEXTURES.get(texture_name, None)
    if texture is not None:
        return texture

    extension, filters = TEXTURES.get(texture_name, ('', ()))
    texture = loader.load_texture(''.join((PATH, texture_name, extension)),
                                  okMissing=True)
    if texture is not None:
        for filter_ in filters:
            filter_(texture)
        LOADED_TEXTURES[texture_name] = texture

    return texture


def set_loader(loader: Loader):
    """Set default loader for use with get."""
    get.loader = loader


def _mipmap(texture: Texture):
    texture.set_minfilter(SamplerState.FT_nearest_mipmap_nearest)


TEXTURES = {
    'ground': TextureInfo('.jpg', (_mipmap,)),
    'intersection': TextureInfo('.png', ()),
    'road': TextureInfo('.png', (_mipmap,))
}
