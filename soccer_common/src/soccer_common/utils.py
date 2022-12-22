import os
from pathlib import Path

import gdown


def download_dataset(url, folder_name):
    # Collect the dataset from the web (https://drive.google.com/drive/u/2/folders/1amhnKBSxHFzkH7op1SusShJckcu-jiUn)
    src_path = os.path.dirname(os.path.realpath(__file__))
    test_path = src_path + f"/../../images/{folder_name}"
    bag_path = Path(test_path)

    if not bag_path.is_dir():
        print(f"Dataset not found at {test_path}. Downloading ...")
        os.makedirs(bag_path)

        zipfilepath = test_path + "/dataset.zip"
        gdown.download(url=url, output=zipfilepath, quiet=False)
        import zipfile

        with zipfile.ZipFile(zipfilepath, "r") as zip_ref:
            zip_ref.extractall(test_path)
        os.remove(zipfilepath)
