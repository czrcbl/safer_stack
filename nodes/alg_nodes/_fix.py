import os
import sys
project_path = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(
                os.path.realpath(__file__)
                )
            )
        )
    )
src_path = os.path.join(project_path, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)
