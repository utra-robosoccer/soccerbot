import * as React from "react";
import Toggle from "react-toggle";
import "./index.scss";
import { map } from "lodash";
import classNames = require("classnames");
import RosmonDisplay from "./rosmon-display";
import { LAUNCH_GROUP_STATE } from "../../../../utils/misc";

interface IDetailsDropdownProps {
  launchGroupState: LAUNCH_GROUP_STATE | undefined;
  launchConfigs: Array<{
    name: string;
    value: string;
    ticked: boolean;
    status: string;
    statename: string | undefined;
  }>;
  robotName: string;
  launchGroup: string;
  changeLaunchGroupOptionsTicked: (
    index: number,
    callback?: () => void
  ) => void;
  shutdownLaunchConfig: (index: number) => void;
}

interface IDetailsDropdownState {
  ticked: boolean[];
  disabled: boolean[];
}

class DetailsDropdown extends React.PureComponent<
  IDetailsDropdownProps,
  IDetailsDropdownState
> {
  public constructor(props: IDetailsDropdownProps) {
    super(props);
    this.state = {
      ticked: map(this.props.launchConfigs, o => o.ticked),
      disabled: map(this.props.launchConfigs, () => false)
    };
  }

  public componentWillReceiveProps(nextProps: IDetailsDropdownProps) {
    this.setState({ ticked: map(nextProps.launchConfigs, o => o.ticked) });
  }

  private static isInError(status: string) {
    return status === "error";
  }

  public render() {
    const {
      robotName,
      launchGroup,
      launchConfigs,
      changeLaunchGroupOptionsTicked,
      shutdownLaunchConfig
    } = this.props;
    const { ticked, disabled } = this.state;
    return (
      <React.Fragment>
        <div className="details-dropdown">
          <div className="details-dropdown-section">
            {map(launchConfigs, (v, index) => (
              <div
                className="launch-config-section"
                key={`${this.props.launchGroup}-${index}`}
              >
                <Toggle
                  checked={ticked[index]}
                  disabled={
                    DetailsDropdown.isInError(v.status) || disabled[index]
                  }
                  onChange={() => {
                    changeLaunchGroupOptionsTicked(index);
                  }}
                />
                <div
                  className={classNames({
                    "launch-config-name": true,
                    "launch-config-name-error": DetailsDropdown.isInError(
                      v.status
                    )
                  })}
                >
                  {v.name}
                </div>
                {DetailsDropdown.isInError(v.status) && (
                  <div
                    className="retry-button"
                    onClick={() => {
                      shutdownLaunchConfig(index);
                    }}
                  >
                    Stop
                  </div>
                )}
              </div>
            ))}
          </div>
          <RosmonDisplay
            launchConfigs={launchConfigs}
            robotName={robotName}
            launchGroup={launchGroup}
          />
        </div>
      </React.Fragment>
    );
  }
}

export default DetailsDropdown;
