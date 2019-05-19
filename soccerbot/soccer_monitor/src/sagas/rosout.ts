import { difference, filter, isNil, map, size } from "lodash";
import configureStore from "../store";
import { END, eventChannel } from "redux-saga";
import { IInitialState } from "../reducers/initial-state";
import { call, cancelled, take } from "redux-saga/effects";
import CONFIG from "../config";
import { ADD_MODIFY_ROSOUT } from "../actions/action-types";
import { dynamicRosoutRead } from "../utils/ros";
import { getAllowPeriodicFetch } from "./periodic-fetch-config";
import { ROSOUT_LOG_LEVEL } from "../utils/misc";

const { configCheckPeriodInMs } = CONFIG;

export interface IAddModifyRosout {
  type: string;
  payload: IInitialState["rosout"][string][number] & {
    robotName: string;
  };
}

export interface IRosoutMessage {
  header: {
    frame_id: string;
    seq: number;
  };
  stamp: {
    secs: number;
    nsecs: number;
  };
  level: 1 | 2 | 4 | 8 | 16;
  name: string;
  msg: string;
  file: string;
  function: string;
  line: number;
  topics: string[];
}

function setStoreChannel() {
  return eventChannel(emmiter => {
    const iv = setInterval(() => {
      if (!isNil(configureStore)) {
        emmiter(configureStore.store);
      }
    }, 0);

    return () => {
      emmiter(END);
      clearInterval(iv);
    };
  });
}

function setRobotNamesChannel(store: typeof configureStore.store | undefined) {
  return eventChannel(emmiter => {
    const iv = setInterval(() => {
      const robots = (store!.getState() as IInitialState).robots;
      emmiter(map(filter(robots, r => r.status === "online"), r => r.name));
    }, configCheckPeriodInMs);

    return () => {
      emmiter(END);
      clearInterval(iv);
    };
  });
}

function* takeAction() {
  const storeChannel = yield call(setStoreChannel);
  let store: typeof configureStore.store | undefined = undefined;

  try {
    while (isNil(store)) {
      store = yield take(storeChannel) || store;
    }
  } finally {
    if (yield cancelled()) {
      storeChannel.close();
    }
  }

  const robotNamesChannel = yield call(setRobotNamesChannel, store);
  try {
    let robotNamesCache: string[] = [];
    while (true) {
      const robotNames = yield take(robotNamesChannel);
      const newRobotNames = difference(robotNames, robotNamesCache);
      if (size(newRobotNames) > 0) {
        robotNamesCache = robotNames;

        yield* map(newRobotNames, function*(robotName: string) {
          const rosoutRead = dynamicRosoutRead(robotName);
          yield rosoutRead.subscribe((message: IRosoutMessage) => {
            if (getAllowPeriodicFetch()) {
              const payload: IAddModifyRosout["payload"] = {
                robotName,
                file: message.file,
                message: message.msg,
                level: ROSOUT_LOG_LEVEL[message.level],
                nodeName: message.name
              };
              store!.dispatch({
                type: ADD_MODIFY_ROSOUT,
                payload
              });
            }
          });
        });
      }
    }
  } finally {
    if (yield cancelled()) {
      storeChannel.close();
    }
  }
}

export function* watchRosout() {
  yield takeAction();
}
