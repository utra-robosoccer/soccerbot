import * as React from "react";
import "./index.scss";
import ProgressBar from "./progress-bar";

interface IStatsProps {
  CPUPercentage: number;
  RAMUsage: number;
  CPUPercentageForProgressBar: number;
  RAMPercentageForProgressBar: number;
  stepSize: number;
  totalCPUCores: number;
  totalRAMSizeInGB: number;
}

const Stats = (props: IStatsProps) => {
  return (
    <React.Fragment>
      <div className="stats">
        <div className="stats-holder-1">
          <div className="section">CPU {`${props.CPUPercentage}%`}</div>
          <div className="section">
            <ProgressBar
              percentage={props.CPUPercentageForProgressBar}
              stepSize={props.stepSize}
            />
          </div>
          <div className="section">
            RAM {`${props.RAMUsage} MiB`}
          </div>
          <div className="section">
            <ProgressBar
              percentage={props.RAMPercentageForProgressBar}
              stepSize={props.stepSize}
            />
          </div>
        </div>
      </div>
    </React.Fragment>
  );
};

export default Stats;
