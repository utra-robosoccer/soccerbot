import CONFIG from "../config";
import axios from "axios";
import { hostname } from "os";

const { apiServerAddress } = CONFIG;

export const removeConfigFile = async (
  hostname: string,
  group: string,
  executable: string
) => {
  return (await axios.delete(
    `${apiServerAddress}/robots/${hostname}/launch_group/${group}/executable/${executable}`
  )).data.response;
};
