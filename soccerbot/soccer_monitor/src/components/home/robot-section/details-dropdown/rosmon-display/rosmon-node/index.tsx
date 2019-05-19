import * as React from "react";
import { isNil } from "lodash";
import "./index.scss";
import { IInitialState } from "../../../../../../reducers/initial-state";
import Stats from "../../../../../stats";
import CONFIG from "../../../../../../config";
import classNames = require("classnames");

const {
  ramSizeInGB,
  numCPUCores,
  rosmonNodeStaleDataTimeoutInMs,
  rosmonNodeStaleDataCheckRateInMs
} = CONFIG;

interface IRosmonNodeProps {
  v: string;
  x: {
    name: string;
    value: string;
    ticked: boolean;
    status: string;
    statename: string | undefined;
  };
  nodeColorClassName:
    | { "status-icon-idle": boolean }
    | { "status-icon-running": boolean }
    | { "status-icon-crashed": boolean }
    | { "status-icon-waiting": boolean };
  rosmonForLaunchGroup: IInitialState["rosmon"][""][""];
}

interface IRosmonNodeState {
  staleData: boolean;
}

class RosmonNode extends React.Component<IRosmonNodeProps, IRosmonNodeState> {
  private timerID: any | null;

  public constructor(props: IRosmonNodeProps) {
    super(props);
    this.state = {
      staleData: false
    };
  }

  public componentDidMount() {
    this.checkForStaleData();
  }

  public componentWillUnmount() {
    if (!isNil(this.timerID)) {
      clearInterval(this.timerID);
    }
  }

  private checkForStaleData = () => {
    const { lastUpdateTimeStamp } = this.props.rosmonForLaunchGroup[
      this.props.x.name
    ][this.props.v];
    const { staleData } = this.state;
    if (Date.now() - lastUpdateTimeStamp > rosmonNodeStaleDataTimeoutInMs) {
      if (!staleData) {
        this.setState({ staleData: true });
      }
    } else {
      if (staleData) {
        this.setState({ staleData: false });
      }
    }

    this.timerID = setTimeout(this.checkForStaleData, rosmonNodeStaleDataCheckRateInMs);
  };

  public render() {
    const { staleData } = this.state;
    return (
      <div
        className={classNames({
          "node-status": true,
          "node-status-stale": staleData
        })}
      >
        <div className="node-status-up">
          <div className="identifier identifier-left">
            <div className="node-name">
              {this.props.v} ({this.props.x.name})
            </div>
          </div>
          <div className="identifier identifier-right">
            <div
              className={classNames({
                "status-icon": true,
                ...this.props.nodeColorClassName
              })}
            />
          </div>
        </div>
        <div className="node-status-down">
          <Stats
            CPUPercentage={
              Math.floor(
                this.props.rosmonForLaunchGroup[this.props.x.name][this.props.v]
                  .stats.cpu * 10000
              ) / 100
            }
            CPUPercentageForProgressBar={
              Math.floor(
                this.props.rosmonForLaunchGroup[this.props.x.name][this.props.v]
                  .stats.cpu * 10000
              ) /
              (numCPUCores * 100)
            }
            totalCPUCores={numCPUCores}
            totalRAMSizeInGB={ramSizeInGB}
            RAMUsage={
              Math.floor(
                (this.props.rosmonForLaunchGroup[this.props.x.name][
                  this.props.v
                ].stats.memory /
                  (1024*1024)) *
                  1000
              ) / 1000
            }
            RAMPercentageForProgressBar={
              Math.floor(
                (this.props.rosmonForLaunchGroup[this.props.x.name][
                  this.props.v
                ].stats.memory /
                  (ramSizeInGB * 10000000)) *
                  100
              ) / 100
            }
            stepSize={100}
          />
        </div>
      </div>
    );
  }
}

export default RosmonNode;
