import { SagaIterator } from "redux-saga";
import { all, call } from "redux-saga/effects";
import {
  watchConfigString,
  watchRobotsInfo,
  watchRobotsInfoPeriodic
} from "./robots-info";
import { watchRosmon } from "./rosmon";
import { watchCmdGlobal } from "./cmd-global";
import { watchRosout } from "./rosout";

export default function* rootSaga(): SagaIterator {
  yield all([
    call(watchRobotsInfo),
    call(watchRobotsInfoPeriodic),
    call(watchConfigString),
    call(watchRosmon),
    call(watchCmdGlobal),
    call(watchRosout)
  ]);
}
