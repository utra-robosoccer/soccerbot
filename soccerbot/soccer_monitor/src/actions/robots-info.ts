import { WATCH_ROBOTS_INFO, WATCH_CONFIG_STRING } from "./action-types";
import { isNil } from "lodash";

export const fetchAndUpdateRobotInfo = (
  forceFetch?: boolean,
  intent?: string
) => ({
  type: WATCH_ROBOTS_INFO,
  payload: {
    forceFetch: !isNil(forceFetch) ? forceFetch : false,
    intent: !isNil(intent) ? intent : ""
  }
});

export const updatePreviousConfigString = (
  hostname: string,
  previousConfigString: string
) => ({
  type: WATCH_CONFIG_STRING,
  payload: { hostname, previousConfigString }
});
