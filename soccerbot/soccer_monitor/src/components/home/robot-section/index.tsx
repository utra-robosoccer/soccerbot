import * as React from "react";
import "./index.scss";
import LaunchGroupsDropdown from "./launch-groups-dropdown";
import {
  cloneDeep,
  filter,
  get,
  includes,
  isEqual,
  isNil,
  map,
  some
} from "lodash";
import DetailsDropdown from "./details-dropdown";
import { launchGroup, removeGroup } from "../../../api/control-group";
import * as robotsInfoActionsUnbound from "../../../actions/robots-info";
import { removeConfigFile } from "../../../api/control-file-config";
import { setAllowPeriodicFetch } from "../../../sagas/periodic-fetch-config";
import * as Noty from "noty";
import { LAUNCH_GROUP_STATE } from "../../../utils/misc";
import classNames = require("classnames");
import RosoutDetails from "./rosout-details";

interface IRobotSectionProps {
  launchGroupOptions:
    | Array<{
        name: string;
        active: boolean;
        value: string;
        launchConfigs: Array<{
          name: string;
          ticked: boolean;
          status: string;
          statename: string;
        }>;
      }>
    | undefined;
  selectedLaunchGroupIndex: number | undefined;
  robotName: string | undefined;
  robotStatus: "online" | "offline" | undefined;
  hostname: string | undefined;
  robotsInfoActions: typeof robotsInfoActionsUnbound;
  previousConfigString: string | undefined;
  currentConfigString: string | undefined;
}

interface IRobotSectionState {
  launchGroupDropdownOpen: boolean;
  detailsDropdownOpen: boolean;
  selectedLaunchGroupIndex: number;
  launchGroupState: LAUNCH_GROUP_STATE | undefined;
  launchGroupOptions: IRobotSectionProps["launchGroupOptions"] | undefined;
  fog: boolean;
}

class RobotSection extends React.PureComponent<
  IRobotSectionProps,
  IRobotSectionState
> {
  private previousLaunchGroupOptions: IRobotSectionState["launchGroupOptions"] = [];
  constructor(props: IRobotSectionProps) {
    super(props);
    this.state = {
      launchGroupDropdownOpen: false,
      detailsDropdownOpen: false,
      selectedLaunchGroupIndex: !isNil(props.selectedLaunchGroupIndex)
        ? props.selectedLaunchGroupIndex
        : 0,
      launchGroupState: LAUNCH_GROUP_STATE.Stopped,
      launchGroupOptions: props.launchGroupOptions,
      fog: false
    };

    if (
      !isNil(this.state.launchGroupOptions) &&
      this.state.launchGroupOptions.length > 0 &&
      !isNil(
        this.state.launchGroupOptions[this.state.selectedLaunchGroupIndex]
          .launchConfigs
      )
    ) {
      (this.state as any).launchGroupState = some(
        this.state.launchGroupOptions[this.state.selectedLaunchGroupIndex]
          .launchConfigs,
        o => o.ticked
      )
        ? LAUNCH_GROUP_STATE.Running
        : LAUNCH_GROUP_STATE.Stopped;
    }
  }

  public componentDidMount() {
    document.body.onclick = (ev: any) => {
      if (
        !includes(
          ["launch-groups-dropdown", "launch-groups-dropdown-entry"],
          ev.target.dataset.value
        )
      ) {
        this.changeLaunchGroupDropdownOpen(false);
      }
    };
  }

  public componentWillUnmount() {
    document.body.onclick = null;
  }

  public componentWillReceiveProps(nextProps: IRobotSectionProps) {
    this.setState({ fog: false });
    const temp = cloneDeep(nextProps.launchGroupOptions);
    const launchGroupOption = temp![nextProps.selectedLaunchGroupIndex!];
    this.setState({ launchGroupOptions: temp });
    if (isNil(launchGroupOption)) {
      return;
    }
    if (some(launchGroupOption.launchConfigs, o => o.ticked)) {
      this.setState({
        launchGroupState: LAUNCH_GROUP_STATE.Running
      });
    } else {
      this.setState({
        launchGroupState: LAUNCH_GROUP_STATE.Stopped
      });
    }
    this.setState({
      selectedLaunchGroupIndex: launchGroupOption.active
        ? nextProps.selectedLaunchGroupIndex!
        : this.state.selectedLaunchGroupIndex
    });
  }

  private onRobotSectionClick = (ev: any) => {
    ev.persist();
    if (
      !includes(
        [
          "launch-groups-dropdown",
          "launch-groups-dropdown-entry",
          "target.dataset.value"
        ],
        get(ev, "target.dataset.value")
      )
    ) {
      this.toggleDetailsDropdownOpen();
      this.changeLaunchGroupDropdownOpen(false);
    }
  };

  private changeLaunchGroupOptionsTicked = async (
    index: number,
    callback?: () => void
  ) => {
    const launchGroupOptions = cloneDeep(this.state.launchGroupOptions);
    const launchGroupOption = launchGroupOptions![
      this.state.selectedLaunchGroupIndex
    ] as {
      name: string;
      value: string;
      active: boolean;
      launchConfigs: Array<{ name: string; ticked: boolean }>;
    };
    const launchConfig = launchGroupOption.launchConfigs[index];
    if (launchConfig.ticked) {
      try {
        setAllowPeriodicFetch(false);
        this.setState({ fog: true });
        await removeConfigFile(
          this.props.hostname!,
          launchGroupOption.name,
          launchConfig.name
        );
      } catch (e) {
        new Noty({
          type: "alert",
          layout: "bottomCenter",
          theme: "nest",
          text: "Error occurred while removing the config file"
        }).show();
        setAllowPeriodicFetch(true);
        this.setState({ fog: false });
        return;
      }
    } else {
      try {
        setAllowPeriodicFetch(false);
        this.setState({ fog: true });
        await launchGroup(
          this.props.hostname!,
          this.state.launchGroupOptions![this.state.selectedLaunchGroupIndex]
            .name,
          [launchConfig.name]
        );
      } catch (e) {
        new Noty({
          type: "alert",
          layout: "bottomCenter",
          theme: "nest",
          text: "Error occurred while launching the config file"
        }).show();
        setAllowPeriodicFetch(true);
        this.setState({ fog: false });
        return;
      }
    }
    this.setState({ launchGroupOptions });
    this.props.robotsInfoActions.fetchAndUpdateRobotInfo(true);
    if (!isNil(callback)) {
      callback();
    }
  };

  private shutdownLaunchConfig = async (index: number) => {
    const launchGroupOptions = cloneDeep(this.state.launchGroupOptions);
    const launchGroupOption = launchGroupOptions![
      this.state.selectedLaunchGroupIndex
    ] as {
      name: string;
      value: string;
      active: boolean;
      launchConfigs: Array<{ name: string; ticked: boolean }>;
    };
    const launchConfig = launchGroupOption.launchConfigs[index];

    try {
      setAllowPeriodicFetch(false);
      this.setState({ fog: true });
      await removeConfigFile(
        this.props.hostname!,
        launchGroupOption.name,
        launchConfig.name
      );
    } catch (e) {
      new Noty({
        type: "alert",
        layout: "bottomCenter",
        theme: "nest",
        text: "Error occurred while launching the config file"
      }).show();
      setAllowPeriodicFetch(true);
      this.setState({ fog: false });
      return;
    }

    this.setState({ launchGroupOptions });
    this.props.robotsInfoActions.fetchAndUpdateRobotInfo(true);
  };

  private toggleLaunchGroupDropdownOpen = () => {
    if (this.state.launchGroupState === LAUNCH_GROUP_STATE.Stopped) {
      this.changeLaunchGroupDropdownOpen(!this.state.launchGroupDropdownOpen);
    }
  };

  private changeLaunchGroupDropdownOpen = (forcedValue: boolean) => {
    this.setState({
      launchGroupDropdownOpen: forcedValue
    });
  };

  private toggleDetailsDropdownOpen = () => {
    this.changeDetailsDropdownOpen(!this.state.detailsDropdownOpen);
  };

  private changeDetailsDropdownOpen = (forcedValue: boolean) => {
    this.setState({
      detailsDropdownOpen: forcedValue
    });
  };

  private changeSelectedLaunchConfigIndex(index: number) {
    this.setState({ selectedLaunchGroupIndex: index });
    this.changeLaunchGroupDropdownOpen(false);
  }

  private getStartButtonName() {
    const launchGroupOption = this.state.launchGroupOptions![
      this.state.selectedLaunchGroupIndex
    ];
    const countErrors = isNil(launchGroupOption)
      ? 0
      : filter(launchGroupOption.launchConfigs, o => o.status === "error")
          .length;
    if (countErrors !== 0) {
      return "Stop All";
    }

    switch (this.state.launchGroupState) {
      case LAUNCH_GROUP_STATE.Starting: {
        return "Stop All";
      }
      case LAUNCH_GROUP_STATE.Stopped: {
        return "Start All";
      }
      case LAUNCH_GROUP_STATE.Running: {
        return "Stop All";
      }
      case LAUNCH_GROUP_STATE.Stopping: {
        return "Stopping";
      }
      default: {
        return "";
      }
    }
  }

  private async handleStartStopPress() {
    const {
      launchGroupState,
      launchGroupOptions,
      selectedLaunchGroupIndex
    } = this.state;
    const launchGroupOption = launchGroupOptions![selectedLaunchGroupIndex];
    const countErrors = isNil(launchGroupOption)
      ? 0
      : filter(launchGroupOption.launchConfigs, o => o.status === "error")
          .length;
    if (
      countErrors !== 0 ||
      includes(
        [LAUNCH_GROUP_STATE.Starting, LAUNCH_GROUP_STATE.Running],
        launchGroupState
      )
    ) {
      this.setState({ fog: true });
      setAllowPeriodicFetch(false);
      await this.stopDeployment();
    } else if (launchGroupState === LAUNCH_GROUP_STATE.Stopped) {
      this.setState({ fog: true });
      setAllowPeriodicFetch(false);
      await this.startDeployment();
    }
  }

  private startDeployment = async () => {
    const oldState = this.state.launchGroupState;
    this.setState({ launchGroupState: LAUNCH_GROUP_STATE.Starting });
    const temp = cloneDeep(this.state.launchGroupOptions);
    const launchGroupOption = temp![this.state.selectedLaunchGroupIndex!];
    // map(launchGroupOption.launchConfigs, o => (o.ticked = true));
    const executables = map(launchGroupOption.launchConfigs, o => o.name);
    try {
      await launchGroup(
        this.props.hostname!,
        launchGroupOption.name,
        executables as string[]
      );
    } catch (e) {
      new Noty({
        type: "alert",
        layout: "bottomCenter",
        theme: "nest",
        text: "Error occurred while launching the group"
      }).show();
      this.setState({ launchGroupState: oldState });
      return;
    }
    this.setState({ launchGroupOptions: temp });
    this.props.robotsInfoActions.fetchAndUpdateRobotInfo(true);
    this.setState({ launchGroupState: LAUNCH_GROUP_STATE.Running });
  };

  private stopDeployment = async () => {
    const oldState = this.state.launchGroupState;
    this.setState({ launchGroupState: LAUNCH_GROUP_STATE.Stopping });
    try {
      await removeGroup(this.props.hostname!);
    } catch (e) {
      new Noty({
        type: "alert",
        layout: "bottomCenter",
        theme: "nest",
        text: "Error occurred while removing the deployment"
      }).show();
      this.setState({ launchGroupState: oldState });
      return;
    }
    this.props.robotsInfoActions.fetchAndUpdateRobotInfo(true);
    this.setState({ launchGroupState: LAUNCH_GROUP_STATE.Stopped });
  };

  private determineConfigChange() {
    const { currentConfigString, previousConfigString } = this.props;
    if (isNil(previousConfigString)) {
      return false;
    } else if (previousConfigString !== currentConfigString) {
      return true;
    }
    return false;
  }

  public render() {
    const {
      launchGroupDropdownOpen,
      selectedLaunchGroupIndex,
      launchGroupState,
      detailsDropdownOpen,
      launchGroupOptions,
      fog
    } = this.state;
    const {
      robotName,
      robotStatus,
      hostname,
      currentConfigString,
      previousConfigString,
      robotsInfoActions
    } = this.props;

    if (!isEqual(launchGroupOptions, this.previousLaunchGroupOptions)) {
      setTimeout(() => this.setState({ fog: false }), 0);
      this.previousLaunchGroupOptions = cloneDeep(
        this.state.launchGroupOptions
      );
    }

    if (this.determineConfigChange()) {
      if (launchGroupState === LAUNCH_GROUP_STATE.Running) {
        return (
          <React.Fragment>
            <div
              className={classNames({
                "robot-config-changed": true,
                "robot-config-changed-disabled": this.state.fog
              })}
            >
              <div className="robot-config-changed-text">
                {robotName}
                's config file changed!
              </div>
              <div className="robot-config-changed-diff">
                <div className="section">
                  <div className="name">Before</div>
                  <pre className="text">{previousConfigString}</pre>
                </div>
                <div className="section">
                  <div className="name">Now</div>
                  <pre className="text">{currentConfigString}</pre>
                </div>
              </div>
              <div className="robot-config-changed-actions">
                <div className="button" onClick={() => location.reload()}>
                  Accept and Reload
                </div>
                <div
                  className="button"
                  onClick={async () => {
                    this.setState({ fog: true });
                    setAllowPeriodicFetch(false);
                    await this.stopDeployment();
                  }}
                >
                  Shutdown
                </div>
              </div>
            </div>
          </React.Fragment>
        );
      } else if (launchGroupState === LAUNCH_GROUP_STATE.Stopped && !fog) {
        robotsInfoActions.updatePreviousConfigString(
          hostname!,
          currentConfigString!
        );
      }
    }

    const showSection =
      !isNil(robotName) &&
      !isNil(hostname) &&
      !isNil(launchGroupOptions) &&
      !isNil(launchGroupOptions[selectedLaunchGroupIndex]) &&
      launchGroupOptions[selectedLaunchGroupIndex].launchConfigs.length > 0 &&
      !isNil(robotStatus);

    if (!showSection) {
      return <div className="robot-info" />;
    }

    return (
      <React.Fragment>
        <div className="robot-info">
          <div className="robot-section">
            <div
              className={classNames({
                "robot-section-up": true,
                "robot-section-disabled": this.state.fog
              })}
              onClick={this.onRobotSectionClick}
            >
              <div className="robot-section-up-block">
                <div className="robot-section-up-block-subsection robot-section-up-block-subsection-left-up">
                  <div className="robot-name">{robotName}</div>
                  {robotStatus === "online" ? (
                    <div className="robot-status robot-status-online" />
                  ) : (
                    <div className="robot-status robot-status-offline" />
                  )}
                </div>
                <div className="robot-section-up-block-subsection robot-section-up-block-subsection-left-down">
                  <div className="hostname">{hostname}</div>
                </div>
              </div>
              <div className="robot-section-up-block">
                <div className="robot-section-up-block-subsection robot-section-up-block-subsection-right-up">
                  {launchGroupOptions!.length > 0 && (
                    <div
                      className={classNames({
                        "launch-groups": true,
                        "launch-groups-disabled": includes(
                          [
                            LAUNCH_GROUP_STATE.Starting,
                            LAUNCH_GROUP_STATE.Running,
                            LAUNCH_GROUP_STATE.Stopping
                          ],
                          launchGroupState
                        )
                      })}
                    >
                      <LaunchGroupsDropdown
                        onClick={this.toggleLaunchGroupDropdownOpen}
                        launchGroupOptions={launchGroupOptions}
                        selectedLaunchGroupIndex={selectedLaunchGroupIndex}
                        launchGroupDropdownOpen={launchGroupDropdownOpen}
                        iteratee={(v, index) => (
                          <div
                            className={classNames({
                              entry: true,
                              "entry-highlighted":
                                index === selectedLaunchGroupIndex
                            })}
                            data-value="launch-groups-dropdown-entry"
                            key={index}
                            onClick={() =>
                              this.changeSelectedLaunchConfigIndex(index)
                            }
                          >
                            {v.name}
                          </div>
                        )}
                      />
                    </div>
                  )}
                </div>
                <div className="robot-section-up-block-subsection robot-section-up-block-subsection-right-down">
                  {!isNil(launchGroupState) && (
                    <div
                      data-value="launch-group-stop-start"
                      className={classNames({
                        "launch-group-stop-start": true,
                        "launch-group-stop-start-stopping":
                          launchGroupState === LAUNCH_GROUP_STATE.Stopping
                      })}
                      onClick={this.handleStartStopPress.bind(this)}
                    >
                      {this.getStartButtonName()}
                    </div>
                  )}
                </div>
              </div>
            </div>
            {detailsDropdownOpen && (
              <div
                className={classNames({
                  "robot-section-down": true,
                  "robot-section-disabled": this.state.fog
                })}
              >
                <DetailsDropdown
                  launchGroup={
                    launchGroupOptions![selectedLaunchGroupIndex].name
                  }
                  robotName={robotName!}
                  launchGroupState={launchGroupState}
                  changeLaunchGroupOptionsTicked={
                    this.changeLaunchGroupOptionsTicked
                  }
                  shutdownLaunchConfig={this.shutdownLaunchConfig}
                  launchConfigs={map(
                    launchGroupOptions![selectedLaunchGroupIndex].launchConfigs,
                    o => ({
                      ...o,
                      value: o.name
                    })
                  )}
                />
                <RosoutDetails robotName={robotName} />
              </div>
            )}
          </div>
        </div>
      </React.Fragment>
    );
  }
}

export default RobotSection;
