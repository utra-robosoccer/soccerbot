import { cmdGlobalRead } from "../utils/ros";
import configureStore from "../store";
import { isNil } from "lodash";
import { IInitialState } from "../reducers/initial-state";

export interface ICmdGlobalMessage {
  cmd: number;
}

export interface IModifyCmdGlobal {
  type: string;
  payload: {
    cmd: number;
  };
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
  setStore();
  yield cmdGlobalRead.subscribe((message: ICmdGlobalMessage) => {
    const currentCmd = (store!.getState() as IInitialState).cmdGlobal;
    if (message.cmd !== currentCmd) {
      store!.dispatch({
        type: "MODIFY_CMD_GLOBAL",
        payload: message
      } as any);
    }
  });
}

export function* watchCmdGlobal() {
  yield takeAction();
}
