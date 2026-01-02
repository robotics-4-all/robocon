from textx import language, metamodel_from_file

from .utils import get_mm


@language('roboconnect', '*.rbr')
def roboconnect_language():
    "roboconnect language"
    mm = get_mm()
    return mm
