import * as React from "react";
import * as classNames from "classnames";
import "./index.scss";
import { map } from "lodash";

interface IProgressBarProps {
  percentage: number;
  stepSize: number;
}

const ProgressBar = (props: IProgressBarProps) => {
  return (
    <React.Fragment>
      <div className="progress-bar">
        {map(
          [
            ...Array(
              (props.stepSize > 0 && props.stepSize) <= 100 ? props.stepSize : 1
            ).keys()
          ],
          v => (
            <div
              key={v}
              className={classNames({
                "bar-block": true,
                filled:
                  props.percentage >=
                  (100 /
                    ((props.stepSize > 0 && props.stepSize) <= 100
                      ? props.stepSize
                      : 1)) *
                    (v + 1)
              })}
            />
          )
        )}
      </div>
    </React.Fragment>
  );
};

export default ProgressBar;
