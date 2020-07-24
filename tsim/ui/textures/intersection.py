import aggdraw
from PIL import Image


def create() -> Image:
    """Generate the texture."""
    image = Image.new('RGBA', (1, 1))
    draw = aggdraw.Draw(image)
    draw.rectangle((0, 0, 1, 1), None, aggdraw.Brush((122, 122, 122)))
    draw.flush()
    # image.show()
    return image
