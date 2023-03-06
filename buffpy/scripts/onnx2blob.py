#! /bin/env python3

import os
import sys
import blobconverter

if len(sys.argv) > 1:
    print(f'Converting blob with blobconverter')

    project_root = os.getenv('PROJECT_ROOT')
    xmlfile = f"{project_root}/buffpy/models/{sys.argv[1]}.xml"
    binfile = f"{project_root}/buffpy/models/{sys.argv[1]}.bin"

    blob_path = blobconverter.from_openvino(
        xml=xmlfile,
        bin=binfile,
        data_type="FP16",
        output_dir=os.path.join(project_root, 'buffpy', 'models'),
        shaves=6,
    )

    print(f'Done')
