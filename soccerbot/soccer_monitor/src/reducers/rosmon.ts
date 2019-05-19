import * as types from "../actions/action-types";
import initialState from "./initial-state";
import { IAddModifyRosmon } from "../sagas/rosmon";
import { set, map, cloneDeep } from "lodash";

export default function rosmonReducer(
  state = initialState.rosmon,
  action: IAddModifyRosmon
) {
  switch (action.type) {
    case types.ADD_MODIFY_ROSMON: {
      const clone = cloneDeep(state);
      map(action.payload, x => {
        const {
          robotName,
          launchGroup,
          launchConfig,
          name,
          state: nodeState,
          restartCount,
          stats,
          lastUpdateTimeStamp
        } = x;

        set(clone, `${robotName}.${launchGroup}.${launchConfig}.${name}`, {
          state: nodeState,
          restartCount,
          lastUpdateTimeStamp,
          stats
        });
      });
      return clone;
    }
    default:
      return state;
  }
}
