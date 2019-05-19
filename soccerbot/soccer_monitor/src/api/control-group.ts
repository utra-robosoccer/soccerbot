import CONFIG from "../config";
import axios from "axios";
import { IRobotInfoResponse } from "./robots-info";
import { ILaunchGroupResponse } from "./control-group";

const { apiServerAddress } = CONFIG;

export type ILaunchGroupResponse = IRobotInfoResponse;

export const launchGroup = async (
  hostname: string,
  group: string,
  executables: string[]
) => {
  return (await axios.post<ILaunchGroupResponse>(
    `${apiServerAddress}/robots/${hostname}/launch_group`,
    {
      group,
      executables
    }
  )).data.response;
};

export const removeGroup = async (hostname: string) => {
  return (await axios.delete(
    `${apiServerAddress}/robots/${hostname}/launch_group`
  )).data.response;
};
