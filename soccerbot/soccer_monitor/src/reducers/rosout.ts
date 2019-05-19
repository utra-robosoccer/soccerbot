import * as types from "../actions/action-types";
import initialState from "./initial-state";
import { size, clone } from "lodash";
import { IAddModifyRosout } from "../sagas/rosout";

export default function rosoutReducer(
  state = initialState.rosout,
  action: IAddModifyRosout
) {
  switch (action.type) {
    case types.ADD_MODIFY_ROSOUT: {
      const { robotName } = action.payload;

      const newRosout = clone(state[robotName]) || [];
      if (size(state[robotName]) === 500) {
        newRosout.shift();
      }
      newRosout.push(action.payload);

      return {
        ...state,
        [robotName]: newRosout
      };
    }
    default:
      return state;
  }
}
