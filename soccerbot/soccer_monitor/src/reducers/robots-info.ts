import * as types from "../actions/action-types";
import initialState from "./initial-state";
import { IAddModifyRobotsInfo } from "../sagas/robots-info";
import { findIndex, cloneDeep, remove, isEqual } from "lodash";

export default function robotsInfoReducer(
  state = initialState.robots,
  action: IAddModifyRobotsInfo
) {
  switch (action.type) {
    case types.ADD_MODIFY_ROBOTS_INFO: {
      const indexToUpdate = findIndex(state, [
        "hostname",
        action.payload.hostname
      ]);
      const clone = cloneDeep(state);
      if (indexToUpdate !== -1) {
        clone[indexToUpdate] = { ...clone[indexToUpdate], ...action.payload };
      } else {
        clone[clone.length] = action.payload;
      }
      return clone;
    }
    case types.REMOVE_ROBOT_INFO: {
      const clone = cloneDeep(state);
      remove(clone, o => o.hostname === action.payload.hostname);
      return clone;
    }
    case types.REMOVE_ROBOT_CONFIG: {
      const indexToUpdate = findIndex(state, [
        "hostname",
        action.payload.hostname
      ]);
      const clone = cloneDeep(state);
      if (indexToUpdate !== -1) {
        clone[indexToUpdate] = {
          name: clone[indexToUpdate].name,
          hostname: clone[indexToUpdate].hostname,
          status: clone[indexToUpdate].status
        };
        return clone;
      }
      return state;
    }
    default:
      return state;
  }
}
