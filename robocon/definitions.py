import os
from os.path import dirname, join


TMP_DIR = os.getenv("TMP_DIR", "/tmp/tdsl")

GRAMMAR_DIR = join(dirname(__file__), "grammar")
