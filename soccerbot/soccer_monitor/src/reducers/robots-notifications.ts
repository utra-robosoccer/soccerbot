import * as types from "../actions/action-types";
import initialState from "./initial-state";
import { IRobotNotification, IRobotRemoveNotification } from "../sagas/robots-info";

export default function robotsNotificationsReducer(
  state = initialState.robotsNotifications,
  action: IRobotNotification | IRobotRemoveNotification
) {
  switch (action.type) {
    case types.ADD_ROBOT_NOTIFICATION: {
      const notificationExists = state.find(
        notification => notification.hash === action.payload.hash
      );
      if (notificationExists) return state;

      const notifications = [...state, action.payload];
      return notifications;
    }
    case types.REMOVE_ROBOT_NOTIFICATION: {
      const { hash } = action.payload;
      const notificationExists = state.find(notification => notification.hash === hash);
      if (!notificationExists) return state;

      const notifications = state.filter(notification => notification.hash !== hash);
      return notifications;
    }
    default:
      return state;
  }
}
