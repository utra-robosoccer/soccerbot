import * as types from "../actions/action-types";
import initialState from "./initial-state";
import { IModifyCmdGlobal } from "../sagas/cmd-global";

export default function cmdGlobalReducer(
  state = initialState.cmdGlobal,
  action: IModifyCmdGlobal
) {
  switch (action.type) {
    case types.MODIFY_CMD_GLOBAL: {
      return action.payload.cmd;
    }
    default:
      return state;
  }
}
