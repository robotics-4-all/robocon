from textx import language

from .utils import get_mm


@language('robocon', '*.rbr')
def robocon_language():
    "robocon language"
    mm = get_mm()
    return mm
