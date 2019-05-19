import { REMOVE_ROBOT_NOTIFICATION } from "./action-types";

export const removeRobotNotification = (hash: number) => ({
  type: REMOVE_ROBOT_NOTIFICATION,
  payload: { hash }
});
