import { getConfigFromURL } from "../utils/data-processing";

export const CONFIG = {
  configFilePath: "/config.yaml",
  robotHostPort: 8000,
  apiServerAddress: "http://localhost:8001",
  configCheckPeriodInMs: 3000,
  rosmonTopicThrottleRateInMs: 0,
  rosoutTopicThrottleRateInMs: 0,
  rosmonRenderRateInMs: 3000,
  rosoutRenderRateInMs: 3000,
  rosmonNodeStaleDataTimeoutInMs: 5000,
  rosmonNodeStaleDataCheckRateInMs: 1000,
  rosoutStaleDataCheckRateInMs: 1000,
  numCPUCores: 4,
  ramSizeInGB: 16,
  rosmonTopic: "ros_monitor",
  cmdGlobalTopic: "cmd_global",
  cmdGlobalInTopic: "cmd_global_in",
  rosBridgeReconnectInitialBackOffInMS: 4000,
  rosBridgeReconnectMaximumBackOffInMS: 90000
};

const CONFIG_FROM_URL = getConfigFromURL();

export default { ...CONFIG, ...CONFIG_FROM_URL };
