import { ADD_MODIFY_ROSMON } from "../actions/action-types";
import { isNil, map } from "lodash";
import configureStore from "../store";
import { getAllowPeriodicFetch } from "./periodic-fetch-config";
import { rosmonRead } from "../utils/ros";

export interface IAddModifyRosmon {
  type: string;
  payload: Array<{
    robotName: string;
    launchGroup: string;
    launchConfig: string;
    name: string;
    state: number;
    lastUpdateTimeStamp: number;
    restartCount: number;
    stats: {
      cpu: number;
      memory: number;
    };
  }>;
}

export interface IRosmonMessage {
  header: {
    frame_id: string;
    seq: number;
  };
  stamp: {
    secs: number;
    nsecs: number;
  };
  launch_config: string;
  launch_group: string;
  nodes: Array<{
    memory: number;
    name: string;
    restart_count: number;
    state: number;
    system_load: number;
    user_load: number;
  }>;
  robot_name: string;
}

let store: typeof configureStore.store | undefined = undefined;

function setStore() {
  if (!isNil(configureStore)) {
    store = configureStore.store;
  } else {
    setTimeout(setStore, 0);
  }
}

function* takeAction() {
  yield rosmonRead.subscribe((message: IRosmonMessage) => {
    setStore();
    if (getAllowPeriodicFetch()) {
      const payload = map(message.nodes, node => ({
        robotName: message.robot_name,
        launchGroup: message.launch_group,
        launchConfig: message.launch_config,
        name: node.name,
        state: node.state,
        lastUpdateTimeStamp: Date.now(),
        restartCount: node.restart_count,
        stats: {
          cpu: node.user_load,
          memory: node.memory
        }
      }));
      store!.dispatch({
        type: ADD_MODIFY_ROSMON,
        payload
      });
    }
  });
}

export function* watchRosmon() {
  yield takeAction();
}
