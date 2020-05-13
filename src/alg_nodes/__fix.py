import os
import sys
path = sys.path.insert(0, os.path.dirname(
    os.path.dirname(
        os.path.abspath(
            os.path.realpath(__file__)
            )
        )
    )
)
if path not in sys.path:
    sys.path.insert(0, path)
