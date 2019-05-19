import { CONFIG } from "../config";
import * as qs from "qs";
import { map, keys, isArray, isObject, isNil, isEmpty } from "lodash";
import crc32 = require("crc-32");
import { IInitialState } from "../reducers/initial-state";

const floatRegex = /^-?\d*\.?\d*$/;
const booleanRegex = /^true$|^false$/;

export const convertObjectKeyTypes = (configObj: any) => {
  map(keys(configObj), key => {
    if (floatRegex.test(configObj[key] as string)) {
      configObj[key] = parseFloat(configObj[key]);
      return;
    }
    if (booleanRegex.test(configObj[key])) {
      configObj[key] = configObj[key] === "true";
      return;
    }
    if (isArray(configObj[key])) {
      setupArray(configObj[key]);
      return;
    }
  });
};

function setupArray(configArr: any) {
  map(configArr, (value, index) => {
    if (floatRegex.test(value)) {
      configArr[index] = parseFloat(value);
      return;
    }
    if (booleanRegex.test(value)) {
      configArr[index] = Boolean(value);
      return;
    }
    if (isArray(value)) {
      setupArray(value);
      return;
    }
    if (isObject(value) && !isArray(value)) {
      convertObjectKeyTypes(value);
      return;
    }
  });
}

export const getConfigFromURL: () => typeof CONFIG = () => {
  const parsedQuery = qs.parse(location.search, {
    ignoreQueryPrefix: true
  }) as typeof CONFIG;
  if (!isNil(parsedQuery)) {
    convertObjectKeyTypes(parsedQuery);
  }
  return parsedQuery;
};

export const getNotificationHash = (
  robotName: IInitialState["robotsNotifications"][0]["robotName"],
  notificationType: IInitialState["robotsNotifications"][0]["type"]
): number => crc32.str(`${robotName}-${notificationType}`);

export const robotsSortOrder = (robot: IInitialState["robots"][0]) => ({
  online: robot.status === "online" ? -1 : 1,
  emptyConfigString: isEmpty(robot.currentConfigString) ? -1 : 1
});
