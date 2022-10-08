#!/usr/bin/env python3

import json
import os
from json import JSONDecodeError

data = {
    "basicTimeStep": 24,
    "ids": "186;196;223;232;254;263;285;8158;8167;8171;8180;8184;8193;8197;8206;8210;8219;8223;9934;17807;17816;17820;17829;17833;17842;17846;17855;17859;17868;17872;19583;19592;19613;19633;19642;19664;19673;19695;27568;27577;27581;27590;27594;27603;27607;27616;27620;27629;27633;29344;37217;37226;37230;37239;37243;37252;37256;37265;37269;37278;37282;37287;37294;37301;37308;38993;39002",
    "labelsIds": "605;606;607;617;612;611;610;614;616;603;602;613;615;604",
    "frames": None,
}

file_path = f"{os.path.join(os.path.dirname(__file__))}/../../external/webots/projects/samples/contests/robocup/controllers/referee/recording.json"
with open(file_path, "r+") as f:
    ff = f.read()
    try:
        jj = json.loads(ff)
        json.dump(jj, f, indent=True)
    except JSONDecodeError as ex:
        frames = json.loads("[" + ff + "]")
        data["frames"] = frames
        with open(file_path, "w") as f2:
            json.dump(data, f2, indent=True)
    pass
