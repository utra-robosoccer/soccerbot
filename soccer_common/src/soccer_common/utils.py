import os
import zipfile
from pathlib import Path

import gdown


def download_dataset(url, folder_path: str):
    # Collect the dataset from the web (https://drive.google.com/drive/u/2/folders/1amhnKBSxHFzkH7op1SusShJckcu-jiUn)
    bag_path = Path(folder_path)

    if not bag_path.is_dir():
        print(f"Dataset not found at {folder_path}. Downloading ...")
        os.makedirs(bag_path)

        zipfilepath = folder_path + "/dataset.zip"
        gdown.download(url=url, output=zipfilepath, quiet=False)

        with zipfile.ZipFile(zipfilepath, "r") as zip_ref:
            zip_ref.extractall(folder_path)
        os.remove(zipfilepath)
