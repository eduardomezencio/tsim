"""Road texture generator."""

import aggdraw
from PIL import Image

SIZE = 64


def create() -> Image:
    """Generate the texture."""
    image = Image.new('RGBA', (SIZE, SIZE))
    draw = aggdraw.Draw(image)
    draw.rectangle((0, 0, SIZE, SIZE), None, aggdraw.Brush((122, 122, 122)))
    for x in (0, SIZE):
        path = aggdraw.Path()
        path.moveto(x, 0)
        path.lineto(x, SIZE)
        draw.path(path, aggdraw.Pen('white', 2))
    draw.flush()
    # image.show()
    return image
