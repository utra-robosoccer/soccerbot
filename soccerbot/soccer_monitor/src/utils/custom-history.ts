import { createBrowserHistory, History, LocationState, Path } from "history";
import * as qs from "qs";
import { trimStart, pick, keys } from "lodash";
import CONFIG from "../config";

const preserveQueryParameters = (
  path: Path,
  queryParameters: string[],
  browserHistory: History
) => {
  const searchObject = qs.parse(trimStart(browserHistory.location.search, "?"));
  return `${path}?${qs.stringify(pick(searchObject, queryParameters))}`;
};

const preserveQueryHistory = (queryParameters: string[]) => {
  const browserHistory = createBrowserHistory();
  const { push: originalPush, replace: originalReplace } = browserHistory;
  (browserHistory as any).push = (path: Path, state?: LocationState) => {
    originalPush(
      preserveQueryParameters(path, queryParameters, browserHistory),
      state
    );
  };
  (browserHistory as any).replace = (path: Path, state?: LocationState) => {
    originalReplace(
      preserveQueryParameters(path, queryParameters, browserHistory),
      state
    );
  };
  return browserHistory;
};

export const history = preserveQueryHistory(keys(CONFIG));
