import * as ROSLib from "roslib";
import CONFIG from "../config";

const {
  cmdGlobalInTopic,
  cmdGlobalTopic,
  rosBridgeReconnectInitialBackOffInMS,
  rosBridgeReconnectMaximumBackOffInMS,
  rosmonTopic,
  rosmonTopicThrottleRateInMs,
  rosoutTopicThrottleRateInMs
} = CONFIG;
const Noty = require("noty");

interface IRosManager {
  rosBridgeURL: string;
  initialBackOff: number;
  maximumBackOff: number;
}

class RosManager {
  public readonly ros: ROSLib.Ros;
  private readonly rosBridgeURL: string;
  private readonly initialBackOff: number;
  private readonly maximumBackOff: number;
  private iterations = 0;
  private currentState: "Disconnected" | "Connected" | "Connecting" | "Error" =
    "Connecting";

  constructor(args: IRosManager) {
    this.rosBridgeURL = args.rosBridgeURL;
    this.ros = new ROSLib.Ros({
      url: this.rosBridgeURL
    });
    this.maximumBackOff = args.maximumBackOff;
    this.initialBackOff = args.initialBackOff;

    this.establishHooks();
  }

  private establishHooks = () => {
    this.ros.on("connection", () => {
      this.currentState = "Connected";
      this.iterations = 0;
    });

    this.ros.on("error", () => {
      if (
        this.currentState === "Connected" ||
        this.currentState === "Connecting"
      ) {
        this.reconnect();
        this.currentState = "Error";
      }
    });

    this.ros.on("close", () => {
      if (
        this.currentState === "Connected" ||
        this.currentState === "Connecting"
      ) {
        this.reconnect();
        this.currentState = "Disconnected";
      }
    });
  };

  private reconnect = () => {
    const timeout = Math.pow(2, this.iterations) * this.initialBackOff;
    if (timeout > this.maximumBackOff) {
      const noty = new Noty({
        text: "Failed to reconnect.<br />Please refresh!",
        type: "error",
        layout: "topRight",
        theme: "nest",
        closeWith: ["button"],
        buttons: [
          Noty.button("REFRESH", "noty-refresh-button", () => {
            location.reload();
          })
        ]
      });
      noty.show();
      return;
    }
    new Noty({
      text: `Trying to reconnect in ${Math.floor(timeout / 1000)} seconds`,
      type: "error",
      timeout: timeout - 1000 > 0 ? timeout - 1000 : timeout,
      progressBar: true,
      layout: "topRight",
      theme: "nest"
    }).show();
    setTimeout(() => {
      this.currentState = "Connecting";
      this.ros.connect(this.rosBridgeURL);
      this.iterations += 1;
    }, timeout);
  };
}

const rosManager = new RosManager({
  rosBridgeURL: ROSBRIDGE_URL,
  initialBackOff: rosBridgeReconnectInitialBackOffInMS,
  maximumBackOff: rosBridgeReconnectMaximumBackOffInMS
});

const ros = rosManager.ros;

export const cmdGlobalRead = new ROSLib.Topic({
  ros,
  name: cmdGlobalTopic,
  messageType: "sootballs_msgs/GlobalCmd"
});

export const cmdGlobalWrite = new ROSLib.Topic({
  ros,
  name: cmdGlobalInTopic,
  messageType: "sootballs_msgs/GlobalCmd"
});

export const rosmonRead = new ROSLib.Topic({
  ros,
  name: rosmonTopic,
  messageType: "rosmon/State",
  queue_length: 1,
  throttle_rate: rosmonTopicThrottleRateInMs
});

export const dynamicRosoutRead = (robotName: string) =>
  new ROSLib.Topic({
    ros,
    name: `${robotName}/rosout_agg`,
    messageType: "rosgraph_msgs/Log",
    queue_length: 1,
    throttle_rate: rosoutTopicThrottleRateInMs
  });

export default ros;
