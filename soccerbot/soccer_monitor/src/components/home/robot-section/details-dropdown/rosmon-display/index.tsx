import * as React from "react";
import { map, isNil, get } from "lodash";
import CONFIG from "../../../../../config";
import configureStore from "../../../../../store";
import "./index.scss";
import { IInitialState } from "../../../../../reducers/initial-state";
import RosmonNode from "./rosmon-node";

const { ramSizeInGB, numCPUCores, rosmonRenderRateInMs } = CONFIG;
const { store } = configureStore;

interface IRosmonDisplayProps {
  launchConfigs: Array<{
    name: string;
    value: string;
    ticked: boolean;
    status: string;
    statename: string | undefined;
  }>;
  robotName: string;
  launchGroup: string;
}

interface IRosmonDisplayState {
  rosmon: IInitialState["rosmon"];
}

class RosmonDisplay extends React.PureComponent<
  IRosmonDisplayProps,
  IRosmonDisplayState
> {
  private setTimeoutId: any;
  constructor(props: IRosmonDisplayProps) {
    super(props);
    this.state = {
      rosmon: {}
    };
  }
  public componentDidMount() {
    this.updateRosmon();
  }

  public componentWillUnmount() {
    clearInterval(this.setTimeoutId);
  }

  private updateRosmon = () => {
    const rosmon = (store.getState() as IInitialState).rosmon;
    this.setState({ rosmon });
    this.setTimeoutId = setTimeout(this.updateRosmon, rosmonRenderRateInMs);
  };

  private getNodeColorClassName = (state: number) => {
    switch (state) {
      case 0: {
        return { "status-icon-idle": true };
      }
      case 1: {
        return { "status-icon-running": true };
      }
      case 2: {
        return { "status-icon-crashed": true };
      }
      case 3: {
        return { "status-icon-waiting": true };
      }
      default: {
        return { "status-icon-idle": true };
      }
    }
  };

  public render() {
    const { launchConfigs, launchGroup, robotName } = this.props;
    const { rosmon } = this.state;
    const rosmonForRobot = rosmon[robotName];
    if (isNil(rosmonForRobot)) {
      return null;
    }
    const rosmonForLaunchGroup = rosmonForRobot[launchGroup];
    return (
      <React.Fragment>
        <div className="details-dropdown-section">
          <div className="overall-info">
            Number of CPU Cores: {numCPUCores} • Total RAM (GB): {ramSizeInGB}
          </div>
          {map(
            launchConfigs,
            (x, index1) =>
              x.ticked && !isNil(get(rosmonForLaunchGroup, x.name))
                ? map(
                    Object.keys(rosmonForLaunchGroup[x.name]),
                    (v, index2) => (
                      <RosmonNode
                        key={`${index1}-${index2}`}
                        v={v}
                        x={x}
                        nodeColorClassName={this.getNodeColorClassName(
                          rosmonForLaunchGroup[x.name][v].state
                        )}
                        rosmonForLaunchGroup={rosmonForLaunchGroup}
                      />
                    )
                  )
                : null
          )}
        </div>
      </React.Fragment>
    );
  }
}

export default RosmonDisplay;
