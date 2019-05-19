import CONFIG from "../config";
import axios from "axios";
import * as jsyaml from "js-yaml";
import { IRobotsStatusResponse } from "./robots-info";
import { map, isNil, some, isEqual } from "lodash";
import { IInitialState } from "../reducers/initial-state";

const { configFilePath, apiServerAddress, robotHostPort } = CONFIG;

export type IRobotsStatusResponse = {
  response: {
    data: Array<{
      name: string;
      hostname: string;
      status: "online" | "offline";
    }>;
  };
  status: string;
};

export interface IRobotsConfigResponse {
  env: {
    ROS_MASTER_URI: string;
  };
  launch_configs: Array<{
    command: string;
    env: object;
    monitor_node: boolean;
    name: string;
  }>;
  launch_groups: Array<{
    name: string;
    launch_configs: string[];
  }>;
  robot_name: string;
}

export type IRobotInfoResponse = {
  response: {
    data: {
      [x: string]: {
        [y: string]: {
          description: string;
          exitstatus: number;
          group: string;
          logfile: string;
          name: string;
          now: number;
          pid: number;
          spawnerr: string;
          start: number;
          state: number;
          stateinfo: string;
          statename: string;
          status: string;
          stderr_logfile: string;
          stdout_logfile: string;
          stop: number;
        };
      };
    };
  };
};

export const getRobotsStatus = async () => {
  return (await axios.get<IRobotsStatusResponse>(`${apiServerAddress}/robots`))
    .data.response;
  // Todo: handle error case
};

export const getRobotConfigMergedInfo = async (
  previousRobotsState: IInitialState["robots"][0],
  forceFetch: boolean
) => {
  const data = (await axios.get(
    `http://${previousRobotsState.hostname}:${robotHostPort}${configFilePath}`
  )).data;
  const robotInfo = (await axios.get<IRobotInfoResponse>(
    `${apiServerAddress}/robots/${previousRobotsState.hostname}`
  )).data.response.data;
  const loadedData = jsyaml.safeLoad(data) as IRobotsConfigResponse;

  const launchConfigNames = map(previousRobotsState.launchGroups, o => o.name);
  const launchConfigNamesFromLoadedData = map(
    loadedData.launch_groups,
    o => o.name
  );

  const rv: {
    launch_groups: Array<{
      name: string;
      active: boolean;
      launch_configs: Array<{
        name: string;
        ticked: boolean;
        status: string;
        statename: string;
      }>;
    }>;
    launch_configs: IRobotsConfigResponse["launch_configs"];
    previousConfigString: string | undefined;
    currentConfigString: string;
  } = {
    launch_groups: [],
    launch_configs: loadedData.launch_configs,
    previousConfigString: isNil(previousRobotsState.previousConfigString)
      ? data
      : previousRobotsState.previousConfigString,
    currentConfigString: data
  };

  function newFetch() {
    map(loadedData.launch_groups, o => {
      rv.launch_groups.push({
        name: o.name,
        active: !isNil(robotInfo[o.name]),
        launch_configs: map(o.launch_configs, c => ({
          name: c,
          status:
            !isNil(robotInfo[o.name]) && !isNil(robotInfo[o.name][c])
              ? robotInfo[o.name][c].status
              : "success",
          ticked:
            !isNil(robotInfo[o.name]) && !isNil(robotInfo[o.name][c])
              ? robotInfo[o.name][c].statename === "RUNNING" ||
                robotInfo[o.name][c].statename === "STARTING"
              : false,
          statename:
            !isNil(robotInfo[o.name]) && !isNil(robotInfo[o.name][c])
              ? robotInfo[o.name][c].statename
              : "STOPPED"
        }))
      });
    });
  }

  if (
    !isEqual(launchConfigNames, []) &&
    !isEqual(launchConfigNames, launchConfigNamesFromLoadedData)
  ) {
    const wasRunning = some(previousRobotsState.launchGroups, g =>
      some(g.launchConfigs, c => c.ticked)
    );
    if (!wasRunning) {
      newFetch();
    } else {
      if (forceFetch) {
        newFetch();
      } else {
        rv.launch_groups = map(previousRobotsState.launchGroups, o => ({
          name: o.name,
          active: o.active,
          launch_configs: o.launchConfigs
        }))!;
      }
    }
  } else {
    newFetch();
  }

  return rv;
  // Todo: handle error case
};
