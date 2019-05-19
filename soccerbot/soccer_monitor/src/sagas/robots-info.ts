import { put, select, takeLatest } from "redux-saga/effects";
import {
  ADD_MODIFY_ROBOTS_INFO,
  ADD_ROBOT_NOTIFICATION,
  REMOVE_ROBOT_NOTIFICATION,
  WATCH_CONFIG_STRING,
  WATCH_ROBOTS_INFO,
  REMOVE_ROBOT_CONFIG
} from "../actions/action-types";
import {
  getRobotConfigMergedInfo,
  getRobotsStatus,
  IRobotsStatusResponse
} from "../api/robots-info";
import { map, findIndex, find, isEqual, isNil, sortBy } from "lodash";
import { IInitialState } from "../reducers/initial-state";
import { delay } from "redux-saga";
import CONFIG from "../config";
import {
  getAllowPeriodicFetch,
  setAllowPeriodicFetch
} from "./periodic-fetch-config";
import { getNotificationHash } from "../utils/data-processing";
import * as Noty from "noty";

const { configCheckPeriodInMs } = CONFIG;
const getState = (state: IInitialState) => state;

const hasDisplayedConfigFetchErrorNotification: {
  [robotName: string]: boolean;
} = {};
let hasDisplayedRobotStatusesFetchErrorNotification = false;

export interface IAddModifyRobotsInfo {
  type: string;
  payload: { hostname: string } & Partial<{
    previousConfigString: string | undefined;
    currentConfigString: string | undefined;
    name: string;
    status: "online" | "offline";
    launchGroups: Array<{
      name: string;
      active: boolean;
      launchConfigs: Array<{
        name: string;
        ticked: boolean;
        status: string;
        statename: string;
      }>;
    }>;
  }>;
}

export interface IRemoveRobotConfig {
  type: string,
  payload: { hostname: string }
};

export interface IRobotNotification {
  type: string,
  payload: IInitialState["robotsNotifications"][0]
};

export interface IRobotRemoveNotification {
  type: string,
  payload: {
    hash: IInitialState["robotsNotifications"][0]["hash"]
  }
};

function* dispatchForReducer(
  allPayloads: IAddModifyRobotsInfo["payload"][],
  action: { type: string; payload: { forceFetch: boolean } }
) {
  yield* map(allPayloads, function*(payload: IAddModifyRobotsInfo["payload"]) {
    if (action.payload.forceFetch || getAllowPeriodicFetch()) {
      yield put({
        type: ADD_MODIFY_ROBOTS_INFO,
        payload
      });
    }
  });
}

function* dispatchForReducerGeneric(
  actions: Array<{ type: string, payload: {} }>
) {
  yield* map(actions, function*({ type, payload }) {
    yield put({ type, payload });
  });
}

function* takeAction1(action: {
  type: string;
  payload: { forceFetch: boolean };
}) {
  if (!(action.payload.forceFetch || getAllowPeriodicFetch())) {
    return;
  }

  try {
    const state = (yield select(getState)) as IInitialState;
    const robotsStatus = (yield getRobotsStatus())
      .data as IRobotsStatusResponse["response"]["data"];
    const allPayloads: IAddModifyRobotsInfo["payload"][] = [];
    const allRemoveConfigPayloads: IRemoveRobotConfig["payload"][] = [];
    const allNotifications: Array<IRobotNotification | IRobotRemoveNotification> = [];

    yield* map(robotsStatus, async robot => {
      const stateIndex = findIndex(
        state.robots,
        o => o.hostname === robot.hostname
      );
      if (robot.status === "online") {
        try {
          const config = await getRobotConfigMergedInfo(
            stateIndex !== -1 ? state.robots[stateIndex] : robot,
            action.payload.forceFetch
          );
          const payload: IAddModifyRobotsInfo["payload"] = {
            hostname: robot.hostname!,
            name: robot.name,
            previousConfigString: config.previousConfigString,
            currentConfigString: config.currentConfigString,
            status: robot.status,
            launchGroups: map(config.launch_groups, g => ({
              name: g.name,
              active: g.active,
              launchConfigs: g.launch_configs
            }))
          };
          if (!isEqual(payload, state.robots[stateIndex])) {
            allPayloads.push(payload);
            hasDisplayedConfigFetchErrorNotification[robot.name] = false;

            const hash = getNotificationHash(payload.name!, "CONFIG_NOT_FOUND")
            const hasNotificationWithHash = find(state.robotsNotifications, ['hash', hash]);
            if (hasNotificationWithHash) {
              allNotifications.push({
                type: REMOVE_ROBOT_NOTIFICATION,
                payload: { hash }
              });
            }
          }
        } catch (e) {
          const payload: IRemoveRobotConfig["payload"] = { hostname: robot.hostname };

          if (stateIndex !== -1) {
            allRemoveConfigPayloads.push(payload);
          }
          else {
            allPayloads.push(robot);
          }

          if (hasDisplayedConfigFetchErrorNotification[robot.name]) return;

          const notificationType = "CONFIG_NOT_FOUND";
          allNotifications.push({
            type: ADD_ROBOT_NOTIFICATION,
            payload: {
              hash: getNotificationHash(robot.name, notificationType),
              robotName: robot.name,
              heading: "Configuration not found",
              content: `Error occurred while fetching the config file for ${robot.name}`,
              type: notificationType,
              level: "error"
            }
          });
          hasDisplayedConfigFetchErrorNotification[robot.name] = true;
        }
      } else {
        if (stateIndex === -1) {
          allPayloads.push(robot);
        } else {
          if (state.robots[stateIndex].status === "online") {
            allPayloads.push(robot);
          }
        }
      }
    });
    if (action.payload.forceFetch && allPayloads.length > 0) {
      yield dispatchForReducer(sortBy(allPayloads, x => x.name), action);
      setAllowPeriodicFetch(true);
    } else if (getAllowPeriodicFetch() && allPayloads.length > 0) {
      yield dispatchForReducer(sortBy(allPayloads, x => x.name), action);
    }

    if (allRemoveConfigPayloads.length) {
      yield dispatchForReducerGeneric(
        allRemoveConfigPayloads.map(payload => ({ type: REMOVE_ROBOT_CONFIG, payload }))
      );
    }

    if (allNotifications.length) {
      yield dispatchForReducerGeneric(allNotifications);
    }
  } catch (e) {
    if (!hasDisplayedRobotStatusesFetchErrorNotification) {
      new Noty({
        type: "alert",
        layout: "bottomCenter",
        theme: "nest",
        text: `Error occurred while fetching the robots' statuses!`
      }).show();
      hasDisplayedRobotStatusesFetchErrorNotification = true;
    }
  }
}

function* takeAction2(action: {
  type: string;
  payload: { hostname: string; previousConfigString: string };
}) {
  yield put({
    type: ADD_MODIFY_ROBOTS_INFO,
    payload: action.payload
  });
}

export function* watchRobotsInfo() {
  yield takeLatest(WATCH_ROBOTS_INFO, takeAction1);
}

export function* watchRobotsInfoPeriodic() {
  while (true) {
    yield takeAction1({
      type: WATCH_ROBOTS_INFO,
      payload: { forceFetch: false }
    });
    yield delay(configCheckPeriodInMs);
  }
}

export function* watchConfigString() {
  yield takeLatest(WATCH_CONFIG_STRING, takeAction2);
}
