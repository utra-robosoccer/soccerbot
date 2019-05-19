import * as React from "react";
import "./index.scss";
import Header from "../header";
import RobotSection from "./robot-section";
import { IInitialState } from "../../reducers/initial-state";
import * as robotsInfoActionsUnbound from "../../actions/robots-info";
import * as robotsNotificationActionsUnbound from "../../actions/robots-notifications";
import { findIndex, isNil, map, sortBy } from "lodash";
import { robotsSortOrder } from "../../utils/data-processing";
import classNames = require("classnames");

interface IHomeProps {
  robots: IInitialState["robots"];
  robotsInfoActions: typeof robotsInfoActionsUnbound;
  robotsNotificationActions: typeof robotsNotificationActionsUnbound;
  robotsNotifications: IInitialState["robotsNotifications"];
  cmdGlobal: IInitialState["cmdGlobal"];
}

interface IHomeState {
  robotSectionTopMargin: number | null;
}

const getSelectedLaunchGroupIndex = (robot: IHomeProps["robots"][0]) => {
  if (isNil(robot.launchGroups) || robot.launchGroups.length <= 0) {
    return 0;
  }
  if (findIndex(robot.launchGroups, o => o.active)! === -1) {
    return 0;
  }
  return findIndex(robot.launchGroups, o => o.active)!;
};

class Home extends React.PureComponent<IHomeProps, IHomeState> {
  constructor(props: IHomeProps) {
    super(props);

    this.state = {
      robotSectionTopMargin: null
    };
  }

  private updateRobotSectionTopMargin = (
    robotSectionTopMargin: IHomeState["robotSectionTopMargin"]
  ) => {
    this.setState({ robotSectionTopMargin });
  };

  public render() {
    const {
      robots,
      robotsInfoActions,
      robotsNotificationActions,
      robotsNotifications,
      cmdGlobal
    } = this.props;
    const { robotSectionTopMargin } = this.state;

    const holderOverrideStyle = {
      marginTop: !isNil(robotSectionTopMargin)
        ? `${robotSectionTopMargin}px`
        : ""
    };

    return (
      <React.Fragment>
        <Header
          cmdGlobal={cmdGlobal}
          updateRobotSectionTopMargin={this.updateRobotSectionTopMargin}
        />
        <div className="robot-section-holder" style={holderOverrideStyle}>
          {map(
            sortBy(
              robots,
              o => robotsSortOrder(o).online,
              o => robotsSortOrder(o).emptyConfigString
            ),
            (robot: IHomeProps["robots"][0], index) => {
              if (
                robot.status === "online" &&
                !isNil(robot.currentConfigString)
              ) {
                return (
                  <RobotSection
                    key={robot.hostname}
                    previousConfigString={robot.previousConfigString}
                    currentConfigString={robot.currentConfigString}
                    robotsInfoActions={robotsInfoActions}
                    hostname={robot.hostname}
                    robotName={robot.name}
                    robotStatus={robot.status}
                    launchGroupOptions={map(robot.launchGroups, group => ({
                      name: group.name,
                      active: group.active,
                      value: group.name,
                      launchConfigs: group.launchConfigs
                    }))}
                    selectedLaunchGroupIndex={getSelectedLaunchGroupIndex(
                      robot
                    )}
                  />
                );
              } else if (robot.status === "online") {
                const notifications = robotsNotifications
                  .filter(notification => notification.robotName === robot.name)
                  .reverse();
                return (
                  <div
                    className="robot-info-disabled robot-info-no-config"
                    key={index}
                  >
                    <div className="robot-info-no-config-content">
                      <div className="hostname">{robot.hostname} (online)</div>
                      <div className="robot-info-no-config-status">
                        Waiting for config...
                      </div>
                    </div>
                    <div
                      className={classNames({
                        "robot-notifications": notifications.length
                      })}
                    >
                      {notifications.map((notification, idx) => (
                        <div
                          className={classNames({
                            "robot-notification": true,
                            "robot-notification-error":
                              notification.level === "error"
                          })}
                          key={idx}
                        >
                          <div className="robot-notification-heading">
                            {notification.heading}
                            <span
                              onClick={() =>
                                robotsNotificationActions.removeRobotNotification(
                                  notification.hash
                                )
                              }
                              className="robot-notification-close"
                              title="Remove this notification"
                            >
                              &times;
                            </span>
                          </div>
                          <div className="robot-notification-body">
                            {notification.content}
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                );
              } else {
                return (
                  <div className="robot-info-disabled" key={index}>
                    <div className="hostname">{robot.hostname} (offline)</div>
                  </div>
                );
              }
            }
          )}
        </div>
      </React.Fragment>
    );
  }
}

export default Home;
