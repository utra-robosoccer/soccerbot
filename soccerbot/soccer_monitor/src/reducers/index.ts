import robotsInfoReducer from "./robots-info";
import robotsNotificationsReducer from "./robots-notifications";
import rosmonReducer from "./rosmon";
import cmdGlobalReducer from "./cmd-global";
import rosoutReducer from "./rosout";

const { combineReducers } = require("redux");

const rootReducer = combineReducers({
  robots: robotsInfoReducer,
  robotsNotifications: robotsNotificationsReducer,
  rosmon: rosmonReducer,
  cmdGlobal: cmdGlobalReducer,
  rosout: rosoutReducer
});

export default rootReducer;
